"""Odometrie-Node für den Pioneer 3-DX auf Basis von Radsensoren.

Abonniert /joint_states, berechnet die Pose via Differential-Drive-
Kinematik (Dead Reckoning) und publiziert nav_msgs/Odometry sowie die
zugehörige odom→base_link TF-Transformation.

Besonderheiten:
  - dt wird ausschließlich aus dem ROS2-Clock berechnet, um
    Zeitquellengemisch (Webots vs. /clock) zu vermeiden.
  - Publiziert auch per Timer mit konfigurierbarer Rate, damit
    Nav2-Komponenten nicht auf Joint-State-Callbacks warten müssen.
  - Große Sprünge in Radpositionen werden auf max_wheel_speed·dt geclampt.

Quellen:
 Kinematikmodell:
   ros2_control Project (2024). Wheeled Mobile Robot Kinematics.
   https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html

 Implementierungsreferenz:
   Aziz, H. (2022). ros2_odometry_estimation [Software].
   https://github.com/hiwad-aziz/ros2_odometry_estimation

"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):
    """Berechnet und publiziert Odometrie aus Differential-Drive-Radsensoren.

    Parameter (nav2_params.yaml oder Launch-File):
      wheel_radius      (float, default: 0.0975)  — Radradius in m.
      wheel_base        (float, default: 0.33)    — Spurweite in m.
      max_wheel_speed   (float, default: 20.0)    — Clamp-Grenze rad/s.
      min_movement      (float, default: 1e-6)    — Rausch-Schwelle in m.
      publish_rate      (float, default: 9.0)     — Timer-Rate in Hz.
      initial_x/y/theta (float, default: 0.0)    — Startzustand.
    """

    def __init__(self):
        super().__init__('odom_publisher')

        # --- Parameter deklarieren und einlesen ---
        self.declare_parameter('wheel_radius',    0.0975)
        self.declare_parameter('wheel_base',      0.33)
        self.declare_parameter('max_wheel_speed', 20.0)
        self.declare_parameter('min_movement',    1e-6)
        self.declare_parameter('publish_rate',    9.0)
        self.declare_parameter('initial_x',       0.0)
        self.declare_parameter('initial_y',       0.0)
        self.declare_parameter('initial_theta',   0.0)

        self._wheel_radius    = self.get_parameter('wheel_radius').value
        self._wheel_base      = self.get_parameter('wheel_base').value
        self._max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self._min_movement    = self.get_parameter('min_movement').value
        self._publish_rate    = self.get_parameter('publish_rate').value

        # --- Pose-Zustand (Dead Reckoning) ---
        self._x     = float(self.get_parameter('initial_x').value)
        self._y     = float(self.get_parameter('initial_y').value)
        self._theta = float(self.get_parameter('initial_theta').value)

        # --- Rad-Tracking ---
        self._prev_left_pos  = None
        self._prev_right_pos = None
        self._last_time_ns   = 0

        # --- Gecachte Werte für Timer-Publikation ---
        self._last_v_linear  = 0.0
        self._last_v_angular = 0.0
        self._last_stamp     = None
        self._have_state     = False

        # --- ROS2-Kommunikation ---
        self.create_subscription(JointState, 'joint_states',  self._joint_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / self._publish_rate, self._timer_cb)

    # -----------------------------------------------------------------------
    # Joint-State-Callback
    # -----------------------------------------------------------------------

    def _joint_cb(self, msg: JointState):
        """Berechnet Odometrie aus neuen Radpositionen.

        Args:
            msg: JointState-Nachricht mit Radpositionen.
        """
        try:
            left_idx  = msg.name.index('left wheel')
            right_idx = msg.name.index('right wheel')
        except ValueError:
            self.get_logger().warning(
                f'Radgelenke nicht in joint_states gefunden. Verfügbar: {msg.name}',
                throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        now_ns = int(getattr(now, 'nanoseconds', 0))
        stamp = now.to_msg()

        # Initialisierung beim ersten Callback.
        if self._prev_left_pos is None:
            try:
                self._prev_left_pos  = msg.position[left_idx]
                self._prev_right_pos = msg.position[right_idx]
            except (IndexError, Exception):
                self.get_logger().warning('JointState-Positionen noch nicht verfügbar.')
                return

            self._last_time_ns = now_ns
            self._last_stamp   = stamp
            self._have_state   = True

            if now_ns != 0:
                self._publish_odom(stamp, 0.0, 0.0)
            return

        # dt aus ROS2-Clock berechnen.
        dt = (now_ns - self._last_time_ns) / 1e9
        if dt <= 0.0:
            return

        curr_left  = msg.position[left_idx]
        curr_right = msg.position[right_idx]

        # Radwinkeländerungen berechnen und auf plausiblen Bereich clampen.
        d_left  = self._clamp_delta(curr_left  - self._prev_left_pos,  dt, 'links')
        d_right = self._clamp_delta(curr_right - self._prev_right_pos, dt, 'rechts')

        # Bogenlängen aus Winkeländerungen.
        left_dist  = d_left  * self._wheel_radius
        right_dist = d_right * self._wheel_radius

        # Rauschen ignorieren.
        if (abs(left_dist) + abs(right_dist)) / 2.0 < self._min_movement:
            self._prev_left_pos  = curr_left
            self._prev_right_pos = curr_right
            self._last_stamp     = stamp
            return

        self._update_pose(left_dist, right_dist)

        # Geschwindigkeiten: aus JointState falls verfügbar, sonst aus Deltas.
        v_left, v_right = self._extract_velocities(
            msg, left_idx, right_idx, left_dist, right_dist, dt)

        self._last_v_linear  = (v_left + v_right) / 2.0
        self._last_v_angular = (v_right - v_left) / self._wheel_base
        self._last_stamp     = stamp
        self._have_state     = True
        self._last_time_ns   = now_ns
        self._prev_left_pos  = curr_left
        self._prev_right_pos = curr_right

        self._publish_odom(stamp, self._last_v_linear, self._last_v_angular)

    # -----------------------------------------------------------------------
    # Timer-Callback
    # -----------------------------------------------------------------------

    def _timer_cb(self):
        """Publiziert Odometrie mit gecachten Werten auf konfigurierbarer Rate."""
        now = self.get_clock().now()
        if getattr(now, 'nanoseconds', 0) == 0:
            return
        if not self._have_state or self._last_stamp is None:
            return

        # Zeitstempel: gecachten Wert nur verwenden wenn er nicht zu alt ist.
        now_ns = int(getattr(now, 'nanoseconds', 0))
        try:
            stamp_ns = int(self._last_stamp.sec) * 10**9 + int(self._last_stamp.nanosec)
        except Exception:  # pylint: disable=broad-except
            stamp_ns = 0

        stamp = (self._last_stamp
                 if stamp_ns != 0 and abs(now_ns - stamp_ns) <= 10**9
                 else now.to_msg())

        self._publish_odom(stamp, self._last_v_linear, self._last_v_angular)

    # -----------------------------------------------------------------------
    # Pose-Update (Differential-Drive-Kinematik)
    # -----------------------------------------------------------------------

    def _update_pose(self, left_dist: float, right_dist: float):
        """Aktualisiert die 2D-Pose via Differential-Drive Dead Reckoning.

        Für gekrümmte Pfade (|Δθ| > 1e-6) wird der Kreisbogen-Ansatz
        verwendet; für gerade Strecken die einfache Näherung.

        Args:
            left_dist:  Gefahrene Strecke des linken Rads in m.
            right_dist: Gefahrene Strecke des rechten Rads in m.
        """
        dist      = (left_dist + right_dist) / 2.0
        d_theta   = (right_dist - left_dist) / self._wheel_base
        prev_theta = self._theta
        self._theta += d_theta

        if abs(d_theta) > 1e-6:
            r = dist / d_theta
            self._x += r * (math.sin(self._theta) - math.sin(prev_theta))
            self._y += r * (math.cos(prev_theta)  - math.cos(self._theta))
        else:
            self._x += dist * math.cos(self._theta)
            self._y += dist * math.sin(self._theta)

    # -----------------------------------------------------------------------
    # Publikation
    # -----------------------------------------------------------------------

    def _publish_odom(self, stamp, v_linear: float, v_angular: float):
        """Publiziert Odometry-Nachricht und odom→base_link TF.

        Falls der übergebene Zeitstempel in der Zukunft liegt (relativ zur
        aktuellen Simulationszeit), wird der aktuelle Clock-Wert verwendet,
        um TF-Extrapolationsfehler zu vermeiden.

        Args:
            stamp:     Kandidaten-Zeitstempel.
            v_linear:  Lineare Geschwindigkeit in m/s.
            v_angular: Winkelgeschwindigkeit in rad/s.
        """
        use_stamp = self._clamp_stamp(stamp)
        orientation = self._yaw_to_quaternion(self._theta)

        # Odometry-Nachricht.
        odom = Odometry()
        odom.header.stamp    = use_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = orientation
        odom.pose.covariance = [
            0.05, 0.0, 0.0,  0.0,  0.0,  0.0,
            0.0, 0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0, 1e6,  0.0,  0.0,  0.0,
            0.0,  0.0, 0.0,  1e6,  0.0,  0.0,
            0.0,  0.0, 0.0,  0.0,  1e6,  0.0,
            0.0,  0.0, 0.0,  0.0,  0.0,  0.2,
        ]

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular
        odom.twist.covariance = [
            0.02, 0.0, 0.0,  0.0,  0.0,  0.0,
            0.0, 0.02, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0, 1e6,  0.0,  0.0,  0.0,
            0.0,  0.0, 0.0,  1e6,  0.0,  0.0,
            0.0,  0.0, 0.0,  0.0,  1e6,  0.0,
            0.0,  0.0, 0.0,  0.0,  0.0,  0.1,
        ]

        self._odom_pub.publish(odom)

        # TF odom → base_link mit identischem Zeitstempel.
        tf = TransformStamped()
        tf.header.stamp          = use_stamp
        tf.header.frame_id       = 'odom'
        tf.child_frame_id        = 'base_link'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation       = orientation
        self._tf_broadcaster.sendTransform(tf)

    # -----------------------------------------------------------------------
    # Hilfsmethoden
    # -----------------------------------------------------------------------

    def _clamp_delta(self, delta: float, dt: float, label: str) -> float:
        """Begrenzt eine Radwinkeländerung auf max_wheel_speed·dt.

        Args:
            delta: Berechnete Winkeländerung in rad.
            dt:    Zeitschritt in s.
            label: Bezeichnung für Log-Ausgaben.

        Returns:
            Geclamper Delta-Wert.
        """
        max_delta = self._max_wheel_speed * dt
        if abs(delta) > max_delta:
            self.get_logger().warning(
                f'Rad {label}: Delta {delta:.3f} rad > {max_delta:.3f} rad, wird geclampt.',
                throttle_duration_sec=2.0)
            return math.copysign(max_delta, delta)
        return delta

    def _extract_velocities(self, msg, left_idx, right_idx,
                             left_dist, right_dist, dt):
        """Liest Radgeschwindigkeiten aus JointState oder berechnet sie aus Deltas.

        Args:
            msg:        JointState-Nachricht.
            left_idx:   Index des linken Rads in msg.name.
            right_idx:  Index des rechten Rads in msg.name.
            left_dist:  Bogenlänge links in m (Fallback).
            right_dist: Bogenlänge rechts in m (Fallback).
            dt:         Zeitschritt in s.

        Returns:
            Tuple (v_left, v_right) in m/s.
        """
        v_left = v_right = None
        if len(msg.velocity) > max(left_idx, right_idx):
            try:
                v_left  = msg.velocity[left_idx]
                v_right = msg.velocity[right_idx]
            except Exception:  # pylint: disable=broad-except
                pass

        if v_left is None:
            v_left  = left_dist  / dt
            v_right = right_dist / dt

        clamp = self._max_wheel_speed
        v_left  = max(-clamp, min(clamp, v_left))
        v_right = max(-clamp, min(clamp, v_right))
        return v_left, v_right

    def _clamp_stamp(self, stamp):
        """Gibt stamp zurück, falls es nicht in der Zukunft liegt.

        Args:
            stamp: Kandidaten-Zeitstempel (ROS2 Time Message).

        Returns:
            Zeitstempel ≤ aktuelle Simulationszeit.
        """
        try:
            stamp_ns = int(stamp.sec) * 10**9 + int(stamp.nanosec)
        except Exception:  # pylint: disable=broad-except
            stamp_ns = 0

        now = self.get_clock().now()
        now_ns = int(getattr(now, 'nanoseconds', 0))

        if stamp_ns == 0 or stamp_ns > now_ns:
            return now.to_msg()
        return stamp

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Konvertiert einen Yaw-Winkel in ein Quaternion (Roll=Pitch=0).

        Args:
            yaw: Rotationswinkel um die z-Achse in rad.

        Returns:
            Quaternion-Nachricht.
        """
        half = yaw / 2.0
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()