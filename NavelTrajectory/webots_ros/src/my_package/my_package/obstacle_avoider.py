"""Pendel-Fussgaenger-Node fuer dynamische Hindernisse.

Faehrt geradeaus und dreht um 180 Grad, sobald ein Hindernis
innerhalb der Wandschwelle erkannt wird. Erzeugt ein vorhersagbares
Hin-und-Her-Verhalten auf einer festen Achse.

Wartet auf ein Signal auf /start_experiment bevor die Bewegung beginnt,
um deterministische Versuchsablaeufe zu ermoeglichen.

Wird pro Fussgaenger-Instanz mit eigenem Namespace gestartet:
  ros2 run my_package obstacle_avoider --ros-args -r __ns:=/person_1
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

# ---------------------------------------------------------------------------
# Modulkonstanten
# ---------------------------------------------------------------------------
_MAX_RANGE = 10.0           # m -- Fallback bei ungueltigen LiDAR-Messwerten
_WALL_THRESHOLD = 0.6       # m -- Abstand ab dem die Drehung ausgeloest wird
_FORWARD_SPEED = 0.4        # m/s -- Normale Vorwaertsgeschwindigkeit
_TURN_SPEED = 1.0           # rad/s -- Drehgeschwindigkeit waehrend 180-Grad-Wende
_TURN_TARGET = math.pi      # rad -- Ziel-Drehwinkel (180 Grad)
_CONTROL_RATE = 20.0        # Hz -- Timer-Frequenz


class ObstacleAvoider(Node):
    """Pendel-Fussgaenger: geradeaus fahren, bei Wand 180 Grad drehen.

    Zustandsautomat:
      IDLE:   Wartet auf /start_experiment Signal (global Topic, kein Namespace).
      DRIVE:  Vorwaertsfahrt mit _FORWARD_SPEED.
              Uebergang zu TURN wenn min(lidar) < _WALL_THRESHOLD.
      TURN:   Drehung auf der Stelle um _TURN_TARGET Radiant.
              Uebergang zu DRIVE wenn Zielwinkel erreicht.
    """

    _STATE_IDLE = 'IDLE'
    _STATE_DRIVE = 'DRIVE'
    _STATE_TURN = 'TURN'

    def __init__(self):
        super().__init__('obstacle_avoider')

        # -- LiDAR-Messwerte --
        self._dist_left = _MAX_RANGE
        self._dist_right = _MAX_RANGE

        # -- Zustandsautomat --
        self._state = self._STATE_IDLE
        self._turn_accumulated = 0.0

        # -- ROS-Interface --
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'lidar_left', self._lidar_left_cb, qos)
        self.create_subscription(LaserScan, 'lidar_right', self._lidar_right_cb, qos)

        # Globales Start-Signal (absolute Topic-Angabe mit fuehrendem /)
        self.create_subscription(Empty, '/start_experiment', self._start_cb, 10)

        self._dt = 1.0 / _CONTROL_RATE
        self.create_timer(self._dt, self._control_cb)

        self.get_logger().info('Pendel-Fussgaenger bereit -- warte auf /start_experiment.')

    # -------------------------------------------------------------------
    # Start-Signal
    # -------------------------------------------------------------------

    def _start_cb(self, msg: Empty):
        """Startet die Bewegung nach Empfang des Experiment-Signals."""
        if self._state == self._STATE_IDLE:
            self._state = self._STATE_DRIVE
            self.get_logger().info('Start-Signal empfangen -- Bewegung gestartet.')

    # -------------------------------------------------------------------
    # LiDAR-Callbacks
    # -------------------------------------------------------------------

    def _lidar_left_cb(self, msg: LaserScan):
        """Aktualisiert den minimalen Abstand des linken LiDAR."""
        self._dist_left = self._min_valid_range(msg)

    def _lidar_right_cb(self, msg: LaserScan):
        """Aktualisiert den minimalen Abstand des rechten LiDAR."""
        self._dist_right = self._min_valid_range(msg)

    # -------------------------------------------------------------------
    # Regelungs-Timer
    # -------------------------------------------------------------------

    def _control_cb(self):
        """Zustandsautomat: IDLE, DRIVE oder TURN."""
        cmd = Twist()

        # -- Zustand: IDLE --
        if self._state == self._STATE_IDLE:
            # Stehen bleiben, nichts publizieren
            self._cmd_pub.publish(cmd)
            return

        combined_min = min(self._dist_left, self._dist_right)

        # -- Zustand: DRIVE --
        if self._state == self._STATE_DRIVE:
            if combined_min < _WALL_THRESHOLD:
                self._state = self._STATE_TURN
                self._turn_accumulated = 0.0
                #self.get_logger().info(
                #    f'Wand bei {combined_min:.2f} m erkannt -- starte 180-Grad-Wende.'
                #)
            else:
                cmd.linear.x = _FORWARD_SPEED
                cmd.angular.z = 0.0

        # -- Zustand: TURN --
        if self._state == self._STATE_TURN:
            if self._turn_accumulated >= _TURN_TARGET:
                self._state = self._STATE_DRIVE
                self._turn_accumulated = 0.0
                #self.get_logger().info('180-Grad-Wende abgeschlossen -- fahre weiter.')
                cmd.linear.x = _FORWARD_SPEED
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = _TURN_SPEED
                self._turn_accumulated += _TURN_SPEED * self._dt

        self._cmd_pub.publish(cmd)

    # -------------------------------------------------------------------
    # Hilfsmethoden
    # -------------------------------------------------------------------

    @staticmethod
    def _min_valid_range(msg: LaserScan) -> float:
        """Gibt den minimalen finiten Messwert eines LaserScan zurueck.

        Args:
            msg: LaserScan-Nachricht.

        Returns:
            Minimaler gueltiger Abstand in Metern.
        """
        if not msg.ranges:
            return _MAX_RANGE
        cleaned = [r if math.isfinite(r) else _MAX_RANGE for r in msg.ranges]
        return min(cleaned)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()