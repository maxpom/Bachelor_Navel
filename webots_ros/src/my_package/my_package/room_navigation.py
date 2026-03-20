"""Waypoint-Guidance-Supervisor für das Navel-Besucherführungssystem.

Navigiert den Roboter sequenziell durch vordefinierte Waypoints und
prüft an konfigurierten Haltepunkten, ob ein Fußgänger folgt
(Motion-Detection-basierter Pedestrian-Check). Bei Contact-Loss wird
eine dreistufige Recovery-Strategie ausgeführt.

Recovery-Strategie bei Contact-Loss an Waypoint N:
  Phase 2: Rückkehr zu Waypoint N-1 + erneuter Pedestrian-Check.
  Phase 3: Falls Phase 2 scheitert oder N=0 → Rückkehr zum Ursprung.

Konfiguration:
  config/room_waypoints.yaml — Raum-Definitionen, Waypoints, Initial Pose.

Subscriptions:
  /go_to_room           (std_msgs/String) — Startet Mission mit Raum-ID.
  /pedestrian_visible   (std_msgs/Bool)   — Bewegungserkennungs-Flag.

"""

import math
import os
import time
from enum import Enum

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# Zustandsmaschine
# ---------------------------------------------------------------------------

class GuidanceState(Enum):
    """Zustände des Guidance-Prozesses."""
    IDLE                   = 0
    NAVIGATING             = 1
    STABILIZING            = 2
    WAITING_FOR_PEDESTRIAN = 3
    TIMEOUT                = 4
    MISSION_COMPLETE       = 5
    RETURNING_TO_PREVIOUS  = 6  # Recovery Phase 2: Rückkehr zu N-1.
    RETURNING_TO_ORIGIN    = 7  # Recovery Phase 3: Rückkehr zum Ursprung.


# ---------------------------------------------------------------------------
# Supervisor-Node
# ---------------------------------------------------------------------------

class MultiWaypointGuidanceSupervisor(Node):
    """Führt Besucher sequenziell durch Waypoints mit Pedestrian-Checks.

    Parameter:
      stabilization_duration       (float, default: 2.5)  — s.
      pedestrian_wait_timeout      (float, default: 10.0) — s.
      motion_check_duration        (float, default: 3.0)  — s.
      motion_detection_threshold   (int,   default: 3)    — Anzahl Events.
      initial_pedestrian_timeout   (float, default: 30.0) — s.
      max_recovery_attempts        (int,   default: 1).
    """

    def __init__(self):
        super().__init__('guidance_supervisor')

        package_dir = get_package_share_directory('my_package')
        config_path = os.path.join(package_dir, 'config', 'room_waypoints.yaml')

        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        self._rooms             = config.get('rooms', {})
        self._initial_pose_cfg  = config.get('initial_pose', None)

        # --- Parameter ---
        self.declare_parameter('stabilization_duration',      2.5)
        self.declare_parameter('pedestrian_wait_timeout',     10.0)
        self.declare_parameter('motion_check_duration',       3.0)
        self.declare_parameter('motion_detection_threshold',  3)
        self.declare_parameter('initial_pedestrian_timeout',  30.0)
        self.declare_parameter('max_recovery_attempts',       1)

        self._stab_duration      = self.get_parameter('stabilization_duration').value
        self._pedestrian_timeout = self.get_parameter('pedestrian_wait_timeout').value
        self._motion_duration    = self.get_parameter('motion_check_duration').value
        self._motion_threshold   = self.get_parameter('motion_detection_threshold').value
        self._initial_timeout    = self.get_parameter('initial_pedestrian_timeout').value
        self._max_recovery       = self.get_parameter('max_recovery_attempts').value

        # --- Mission-Zustand ---
        self._state                    = GuidanceState.IDLE
        self._current_waypoints        = []
        self._current_waypoint_index   = 0
        self._current_room_id          = None

        # --- Motion-Detection-Zustand ---
        self._motion_count             = 0
        self._motion_start_time        = None
        self._accepting_motion         = False
        self._initial_check_mode       = False

        # --- Recovery-Zustand ---
        self._in_recovery              = False
        self._recovery_attempts        = 0
        self._waypoint_before_loss     = None

        # --- Timer-Handles (werden bei Bedarf erstellt und gecancelt) ---
        self._stab_timer               = None
        self._timeout_timer            = None
        self._pedestrian_check_timer   = None

        # --- Origin Pose aus Konfiguration extrahieren ---
        self._origin_pose = None
        if self._initial_pose_cfg:
            self._origin_pose = {
                'position':    self._initial_pose_cfg['position'],
                'yaw':         self._initial_pose_cfg['yaw'],
                'description': 'Ursprungsort (Initial Pose)',
            }

        # --- ROS2-Kommunikation ---
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 1)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(String, '/go_to_room',          self._go_to_room_cb,    10)
        self.create_subscription(Bool,   '/pedestrian_visible',  self._pedestrian_cb,    10)

        self._reset_motion_client = self.create_client(
            Trigger, '/reset_motion_detection')

        if self._reset_motion_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('✓ Motion Detection Service verfügbar.')
        else:
            self.get_logger().warn(
                'Motion Detection Service nicht verfügbar! '
                'Starte: ros2 run navel motion_detector')

        # Initial Pose einmalig nach 3 s publizieren.
        self._initial_pose_published = False
        self.create_timer(3.0, self._publish_initial_pose_once)

        self._log_startup()

    # -----------------------------------------------------------------------
    # Initialisierung
    # -----------------------------------------------------------------------

    def _log_startup(self):
        """Gibt Konfigurationsparameter beim Start aus."""
        sep = '=' * 70
        self.get_logger().info(sep)
        self.get_logger().info('Room Navigation Supervisor initialisiert')
        self.get_logger().info(sep)
        self.get_logger().info(f'  Stabilisierung:      {self._stab_duration} s')
        self.get_logger().info(f'  Pedestrian-Timeout:  {self._pedestrian_timeout} s')
        self.get_logger().info(f'  Motion-Check:        {self._motion_duration} s')
        self.get_logger().info(f'  Motion-Threshold:    {self._motion_threshold}x')
        self.get_logger().info(f'  Initial-Timeout:     {self._initial_timeout} s')
        self.get_logger().info(f'  Max-Recovery:        {self._max_recovery}')
        self.get_logger().info(sep)

    def _publish_initial_pose_once(self):
        """Publiziert die initiale AMCL-Pose einmalig aus der YAML-Konfiguration."""
        if self._initial_pose_published:
            return

        if self._initial_pose_cfg is None:
            self.get_logger().warn('Keine initial_pose in YAML definiert.')
            self._initial_pose_published = True
            return

        pos  = self._initial_pose_cfg['position']
        quat = _yaw_to_quaternion(self._initial_pose_cfg['yaw'])

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id        = 'map'
        msg.header.stamp           = self.get_clock().now().to_msg()
        msg.pose.pose.position.x   = float(pos['x'])
        msg.pose.pose.position.y   = float(pos['y'])
        msg.pose.pose.position.z   = float(pos.get('z', 0.0))
        msg.pose.pose.orientation.x = quat['x']
        msg.pose.pose.orientation.y = quat['y']
        msg.pose.pose.orientation.z = quat['z']
        msg.pose.pose.orientation.w = quat['w']

        self._initial_pose_pub.publish(msg)
        self._initial_pose_published = True

    # -----------------------------------------------------------------------
    # Subscription-Callbacks
    # -----------------------------------------------------------------------

    def _go_to_room_cb(self, msg: String):
        """Startet eine Guidance-Mission zu einem konfigurierten Raum.

        Args:
            msg: String-Nachricht mit der Raum-ID (z.B. 'office_101').
        """
        if not self._initial_pose_published:
            self.get_logger().warn('AMCL noch nicht initialisiert.')
            return

        if self._state != GuidanceState.IDLE:
            self.get_logger().warn(
                f'Navigation bereits aktiv (State: {self._state.name}).')
            return

        room_id = msg.data
        if room_id not in self._rooms:
            self.get_logger().error(f'Raum "{room_id}" unbekannt.')
            return

        waypoints = self._rooms[room_id].get('waypoints', [])
        if not waypoints:
            self.get_logger().error(f'Raum "{room_id}" hat keine Waypoints.')
            return

        self._current_room_id          = room_id
        self._current_waypoints        = waypoints
        self._current_waypoint_index   = 0
        self._in_recovery              = False
        self._recovery_attempts        = 0
        self._waypoint_before_loss     = None

        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Mission: {self._rooms[room_id]["name"]}')
        self.get_logger().info(f'Waypoints: {len(waypoints)}')
        self.get_logger().info('=' * 70)

        self._initial_check_mode = True
        self._start_stabilization()

    def _pedestrian_cb(self, msg: Bool):
        """Zählt Bewegungserkennungs-Events während aktivem Check.

        Args:
            msg: Bool-Flag von /pedestrian_visible.
        """
        if not self._accepting_motion:
            return
        if self._state == GuidanceState.WAITING_FOR_PEDESTRIAN and msg.data:
            self._motion_count += 1
            self.get_logger().info(
                f'Bewegung erkannt ({self._motion_count}x)',
                throttle_duration_sec=0.5)

    # -----------------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------------

    def _navigate_to_waypoint(self, waypoint: dict, is_recovery: bool = False):
        """Schickt ein NavigateToPose-Goal für einen Waypoint ab.

        Args:
            waypoint:    Waypoint-Dictionary mit 'position', 'yaw', 'description'.
            is_recovery: True wenn im Recovery-Mode navigiert wird.
        """
        pos  = waypoint['position']
        quat = _yaw_to_quaternion(waypoint['yaw'])
        desc = waypoint.get(
            'description',
            f'Waypoint {self._current_waypoint_index + 1}')

        prefix = '[RECOVERY] ' if is_recovery else ''
        self.get_logger().info(
            f'{prefix}→ Waypoint '
            f'{self._current_waypoint_index + 1}/{len(self._current_waypoints)}: {desc}')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id   = 'map'
        goal.pose.header.stamp      = self.get_clock().now().to_msg()
        goal.pose.pose.position.x   = float(pos['x'])
        goal.pose.pose.position.y   = float(pos['y'])
        goal.pose.pose.position.z   = float(pos.get('z', 0.0))
        goal.pose.pose.orientation.x = quat['x']
        goal.pose.pose.orientation.y = quat['y']
        goal.pose.pose.orientation.z = quat['z']
        goal.pose.pose.orientation.w = quat['w']

        self._state = GuidanceState.NAVIGATING
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _navigate_to_current_waypoint(self):
        """Navigiert zum aktuellen Waypoint (self._current_waypoint_index)."""
        if self._current_waypoint_index >= len(self._current_waypoints):
            self.get_logger().error('Waypoint-Index außerhalb des Bereichs.')
            return
        self._navigate_to_waypoint(
            self._current_waypoints[self._current_waypoint_index],
            is_recovery=self._in_recovery)

    # -----------------------------------------------------------------------
    # Action-Callbacks
    # -----------------------------------------------------------------------

    def _goal_response_cb(self, future):
        """Verarbeitet die Goal-Acceptance des NavigateToPose-Actions.

        Args:
            future: Future-Objekt mit GoalHandle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation Goal abgelehnt.')
            self._reset_mission()
            return
        goal_handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        """Verarbeitet das Ergebnis eines abgeschlossenen NavigateToPose-Goals.

        Args:
            future: Future-Objekt mit dem Action-Ergebnis.
        """
        waypoint = self._current_waypoints[self._current_waypoint_index]
        desc = waypoint.get(
            'description',
            f'Waypoint {self._current_waypoint_index + 1}')
        self.get_logger().info(f'✓ Waypoint "{desc}" erreicht.')

        if self._state == GuidanceState.RETURNING_TO_PREVIOUS:
            self.get_logger().info('=' * 70)
            self.get_logger().info('RECOVERY PHASE 2: Pedestrian-Check bei N-1')
            self.get_logger().info('=' * 70)
            self._start_stabilization()
            return

        if waypoint.get('check_pedestrian', False):
            self.get_logger().info('Starte Pedestrian-Check.')
            self._start_stabilization()
        else:
            self._proceed_to_next_waypoint()

    # -----------------------------------------------------------------------
    # Stabilisierung & Pedestrian-Check
    # -----------------------------------------------------------------------

    def _start_stabilization(self):
        """Pausiert Motion-Detection für stab_duration Sekunden."""
        self._state            = GuidanceState.STABILIZING
        self._accepting_motion = False
        self.get_logger().info(
            f'Stabilisierung ({self._stab_duration} s) — Motion Detection pausiert.')
        self._stab_timer = self.create_timer(
            self._stab_duration, self._stabilization_done_cb)

    def _stabilization_done_cb(self):
        """Wird nach Ablauf der Stabilisierungsphase aufgerufen."""
        self._cancel_timer('_stab_timer')
        self.get_logger().info('Stabilisierung abgeschlossen.')

        if not self._reset_motion_detector():
            self.get_logger().warn('Motion Detector Reset fehlgeschlagen.')

        time.sleep(0.2)

        timeout = (self._initial_timeout
                   if self._initial_check_mode else self._pedestrian_timeout)

        self._state            = GuidanceState.WAITING_FOR_PEDESTRIAN
        self._accepting_motion = True
        self._motion_count     = 0
        self._motion_start_time = self.get_clock().now()

        self._pedestrian_check_timer = self.create_timer(
            0.5, self._pedestrian_check_cb)
        self._timeout_timer = self.create_timer(
            timeout, self._pedestrian_timeout_cb)

        self.get_logger().info(
            f'Motion Detection aktiv ({self._motion_duration} s)...')

    def _pedestrian_check_cb(self):
        """Überprüft periodisch, ob genug Bewegungsereignisse eingegangen sind."""
        if self._state != GuidanceState.WAITING_FOR_PEDESTRIAN:
            self._cancel_timer('_pedestrian_check_timer')
            return

        elapsed = (
            self.get_clock().now() - self._motion_start_time
        ).nanoseconds / 1e9

        if self._motion_count >= self._motion_threshold:
            self._cancel_timer('_timeout_timer')
            self._cancel_timer('_pedestrian_check_timer')
            self._accepting_motion = False
            self.get_logger().info(
                f'✓ Pedestrian erkannt ({self._motion_count} Bewegungen).')
            self._on_pedestrian_found()
            return

        # Wartezeit abgelaufen ohne genug Bewegung → Fenster verlängern.
        if elapsed > self._motion_duration and self._motion_count > 0:
            self.get_logger().warn(
                f'Nur {self._motion_count}/{self._motion_threshold} '
                f'Bewegungen — verlängere Wartezeit.')
            self._motion_start_time = self.get_clock().now()
            self._motion_count      = 0

    def _on_pedestrian_found(self):
        """Verarbeitet einen erfolgreichen Pedestrian-Check."""
        if self._in_recovery:
            self.get_logger().info('=' * 70)
            self.get_logger().info('RECOVERY ERFOLGREICH — Pedestrian bei N-1 gefunden.')
            self.get_logger().info('=' * 70)
            self._in_recovery          = False
            self._recovery_attempts    = 0
            self._waypoint_before_loss = None
            self._state = GuidanceState.IDLE
            self._proceed_to_next_waypoint()
            return

        if self._initial_check_mode:
            self.get_logger().info('=' * 70)
            self.get_logger().info('Mission startet.')
            self.get_logger().info('=' * 70)
            self._initial_check_mode = False
            self._state = GuidanceState.IDLE
            self._navigate_to_current_waypoint()
        else:
            self._proceed_to_next_waypoint()

    def _pedestrian_timeout_cb(self):
        """Timeout-Handler — implementiert die dreistufige Recovery-Strategie."""
        self._cancel_timer('_timeout_timer')
        self._cancel_timer('_pedestrian_check_timer')
        self._accepting_motion = False

        # Fall 1: Initial-Check vor Missionsbeginn.
        if self._initial_check_mode:
            self.get_logger().error('=' * 70)
            self.get_logger().error(
                f'INITIAL CHECK TIMEOUT nach {self._initial_timeout} s — '
                f'Mission wird NICHT gestartet.')
            self.get_logger().error('=' * 70)
            self._initial_check_mode = False
            self._reset_mission()
            return

        # Fall 2: Contact-Loss während Mission (nicht im Recovery).
        if not self._in_recovery:
            self.get_logger().error('=' * 70)
            self.get_logger().error(
                f'CONTACT LOSS an Waypoint {self._current_waypoint_index + 1} '
                f'nach {self._pedestrian_timeout} s.')
            self.get_logger().error('=' * 70)
            self._waypoint_before_loss = self._current_waypoint_index
            self._recovery_attempts   += 1

            if self._current_waypoint_index == 0:
                self.get_logger().warn(
                    'Contact-Loss am ersten Waypoint → direkt zum Ursprung.')
                self._return_to_origin()
            else:
                self._return_to_previous_waypoint()
            return

        # Fall 3: Recovery fehlgeschlagen.
        self.get_logger().error('=' * 70)
        self.get_logger().error(
            'RECOVERY FEHLGESCHLAGEN — Pedestrian auch bei N-1 nicht gefunden.')
        self.get_logger().error('→ Rückkehr zum Ursprung.')
        self.get_logger().error('=' * 70)
        self._return_to_origin()

    # -----------------------------------------------------------------------
    # Recovery
    # -----------------------------------------------------------------------

    def _return_to_previous_waypoint(self):
        """Recovery Phase 2: Navigiert zu Waypoint N-1 für erneuten Check."""
        if self._current_waypoint_index <= 0:
            self.get_logger().error('N < 0 — kein vorheriger Waypoint vorhanden.')
            self._return_to_origin()
            return

        self._current_waypoint_index -= 1
        prev_wp = self._current_waypoints[self._current_waypoint_index]

        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'RECOVERY PHASE 2: Rückkehr zu Waypoint '
            f'{self._current_waypoint_index + 1} — '
            f'{prev_wp.get("description", "N-1")}')
        self.get_logger().info('=' * 70)

        self._in_recovery = True
        self._state       = GuidanceState.RETURNING_TO_PREVIOUS
        self._navigate_to_waypoint(prev_wp, is_recovery=True)

    def _return_to_origin(self):
        """Recovery Phase 3: Navigiert zum Ursprungsort und beendet die Mission."""
        if self._origin_pose is None:
            self.get_logger().error('Origin Pose nicht definiert — kann nicht zurückkehren.')
            self._reset_mission()
            return

        self.get_logger().info('=' * 70)
        self.get_logger().info('RECOVERY PHASE 3: Rückkehr zum Ursprung.')
        self.get_logger().info('Mission wird nach Ankunft beendet.')
        self.get_logger().info('=' * 70)

        pos  = self._origin_pose['position']
        quat = _yaw_to_quaternion(self._origin_pose['yaw'])

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id    = 'map'
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = float(pos['x'])
        goal.pose.pose.position.y    = float(pos['y'])
        goal.pose.pose.position.z    = float(pos.get('z', 0.0))
        goal.pose.pose.orientation.x = quat['x']
        goal.pose.pose.orientation.y = quat['y']
        goal.pose.pose.orientation.z = quat['z']
        goal.pose.pose.orientation.w = quat['w']

        self._state = GuidanceState.RETURNING_TO_ORIGIN
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._origin_response_cb)

    def _origin_response_cb(self, future):
        """Verarbeitet Goal-Acceptance für den Origin-Return.

        Args:
            future: Future-Objekt mit GoalHandle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Origin Navigation Goal abgelehnt.')
            self._reset_mission()
            return
        goal_handle.get_result_async().add_done_callback(self._origin_result_cb)

    def _origin_result_cb(self, future):
        """Beendet die Mission nach Ankunft am Ursprungsort.

        Args:
            future: Future-Objekt mit Action-Ergebnis.
        """
        self.get_logger().info('=' * 70)
        self.get_logger().info('✓ Ursprungsort erreicht — Mission abgebrochen.')
        self.get_logger().info('=' * 70)
        self._state = GuidanceState.TIMEOUT
        self._reset_mission()

    # -----------------------------------------------------------------------
    # Mission-Steuerung & Hilfsmethoden
    # -----------------------------------------------------------------------

    def _proceed_to_next_waypoint(self):
        """Wechselt zum nächsten Waypoint oder beendet die Mission."""
        self._current_waypoint_index += 1

        if self._current_waypoint_index < len(self._current_waypoints):
            self.get_logger().info('Nächster Waypoint...')
            self._navigate_to_current_waypoint()
        else:
            room_name = self._rooms[self._current_room_id]['name']
            self.get_logger().info('=' * 70)
            self.get_logger().info(f'✓ Mission abgeschlossen: {room_name}')
            self.get_logger().info('=' * 70)
            self._state = GuidanceState.MISSION_COMPLETE
            self._reset_mission()

    def _reset_mission(self):
        """Setzt den gesamten Mission-Zustand zurück → IDLE."""
        self._accepting_motion = False

        for attr in ('_stab_timer', '_timeout_timer', '_pedestrian_check_timer'):
            self._cancel_timer(attr)

        self._current_waypoints      = []
        self._current_waypoint_index = 0
        self._current_room_id        = None
        self._motion_count           = 0
        self._motion_start_time      = None
        self._in_recovery            = False
        self._recovery_attempts      = 0
        self._waypoint_before_loss   = None
        self._state                  = GuidanceState.IDLE

        self.get_logger().info('Bereit für neue Mission.')

    def _cancel_timer(self, attr: str):
        """Cancelt einen benannten Timer-Handle sicher.

        Args:
            attr: Attributname des Timers (z.B. '_stab_timer').
        """
        timer = getattr(self, attr, None)
        if timer is not None:
            timer.cancel()
            setattr(self, attr, None)

    def _reset_motion_detector(self) -> bool:
        """Ruft den /reset_motion_detection Service auf.

        Returns:
            True bei Erfolg, False bei Timeout oder Fehler.
        """
        self.get_logger().info('Resette Motion Detector...')
        future = self._reset_motion_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            self.get_logger().error('Service Call Timeout.')
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(f'Service fehlgeschlagen: {response.message}')
            return False

        self.get_logger().info(f'Motion Detector Reset: {response.message}')
        return True


# ---------------------------------------------------------------------------
# Modul-Hilfsfunktion
# ---------------------------------------------------------------------------


def _yaw_to_quaternion(yaw: float) -> dict:
    """Konvertiert einen Yaw-Winkel in ein Quaternion.

    q = [0, 0, sin(yaw/2), cos(yaw/2)]

    Args:
        yaw: Rotationswinkel um die z-Achse in rad.

    Returns:
        Dictionary mit x, y, z, w Komponenten.
    """
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = MultiWaypointGuidanceSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()