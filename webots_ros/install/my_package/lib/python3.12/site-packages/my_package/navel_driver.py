"""Webots-Treiber für den Pioneer 3-DX Hauptroboter (NavelDriver).

Wird von webots_ros2_driver als Plugin geladen. Implementiert
Differential-Drive-Kinematik und publiziert JointStates.

Quellen:
  [WEBOTS_DRV] Open Navigation LLC (2024). Setting Up Webots Basic Simulation.
               https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/
               Webots/Setting-Up-Simulation-Webots-Basic.html
"""

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# Kinematik-Konstanten (Pioneer 3-DX).
_HALF_WHEEL_SEPARATION = 0.165  # m  (Spurweite 0.33 m / 2)
_WHEEL_RADIUS = 0.0975           # m


class NavelDriver:
    """Webots-Plugin-Klasse für den Pioneer 3-DX.

    Die Methoden `init` und `step` werden von webots_ros2_driver aufgerufen.
    `init` ersetzt `__init__`, da das Plugin-Interface keinen Standard-
    Konstruktor unterstützt.
    """

    def init(self, webots_node, properties):
        """Initialisiert Motoren, Sensoren und ROS2-Kommunikation.

        Args:
            webots_node: WebotsController-Instanz (Webots Robot + ROS2 Node).
            properties:  Plugin-Properties aus der URDF (ungenutzt).
        """
        self._robot = webots_node.robot
        self._ros_node = self._resolve_ros_node(webots_node)

        # Motoren konfigurieren: Geschwindigkeitsregelung (Position = inf).
        self._left_motor = self._robot.getDevice('left wheel')
        self._right_motor = self._robot.getDevice('right wheel')

        if not self._left_motor or not self._right_motor:
            print('[NavelDriver] ERROR: Radmotoren nicht gefunden!')
            return

        for motor in (self._left_motor, self._right_motor):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Radsensoren für Odometrie aktivieren.
        timestep = int(self._robot.getBasicTimeStep())
        self._left_sensor = self._enable_sensor('left wheel sensor', timestep)
        self._right_sensor = self._enable_sensor('right wheel sensor', timestep)

        # ROS2-Kommunikation.
        self._target_twist = Twist()
        self._cmd_timeout = 0.5   # s — Stop bei ausbleibendem cmd_vel
        self._last_cmd_time = None

        self._ros_node.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 1)
        self._ros_node.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 1)

        self._joint_state_pub = self._ros_node.create_publisher(
            JointState, '/joint_states', 10)

        print('[NavelDriver] Initialisierung abgeschlossen.')

    # -----------------------------------------------------------------------
    # Webots Step-Callback
    # -----------------------------------------------------------------------

    def step(self):
        """Wird jeden Simulations-Zeitschritt von Webots aufgerufen."""
        if rclpy.ok():
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)

        self._publish_joint_states()
        self._apply_velocity()

    # -----------------------------------------------------------------------
    # Private Hilfsmethoden
    # -----------------------------------------------------------------------

    def _resolve_ros_node(self, webots_node):
        """Gibt den eingebetteten ROS2-Node zurück oder erstellt einen Fallback.

        Args:
            webots_node: WebotsController-Instanz.

        Returns:
            rclpy.node.Node-Instanz.
        """
        if hasattr(webots_node, '_WebotsController__node'):
            return webots_node._WebotsController__node
        if hasattr(webots_node, 'robot_node'):
            return webots_node.robot_node

        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node('navel_driver')
        print('[NavelDriver] Fallback: eigener ROS2-Node erstellt.')
        return node

    def _enable_sensor(self, name, timestep):
        """Aktiviert einen Webots-Sensor und gibt ihn zurück.

        Args:
            name:     Gerätename im Webots-World-File.
            timestep: Simulationszeitschritt in ms.

        Returns:
            Aktivierter Sensor oder None.
        """
        sensor = self._robot.getDevice(name)
        if sensor:
            sensor.enable(timestep)
        return sensor

    def _cmd_vel_callback(self, twist):
        """Speichert den aktuellen Geschwindigkeitsbefehl.

        Args:
            twist: Eingehende Twist-Nachricht von /cmd_vel.
        """
        self._target_twist = twist
        try:
            self._last_cmd_time = self._ros_node.get_clock().now()
        except Exception:  # pylint: disable=broad-except
            self._last_cmd_time = None

    def _publish_joint_states(self):
        """Publiziert die aktuellen Radpositionen als JointState-Nachricht."""
        if not (self._left_sensor and self._right_sensor):
            return

        now = self._ros_node.get_clock().now()
        if getattr(now, 'nanoseconds', 0) == 0:
            return  # /clock noch nicht bereit.

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ''
        msg.name = ['left wheel', 'right wheel']
        msg.position = [
            self._left_sensor.getValue(),
            self._right_sensor.getValue(),
        ]
        msg.velocity = []
        msg.effort = []
        self._joint_state_pub.publish(msg)

    def _apply_velocity(self):
        """Berechnet Radgeschwindigkeiten und schreibt sie auf die Motoren.

        Differential-Drive Inverse Kinematik:
          v_l = (v - ω·d) / r
          v_r = (v + ω·d) / r
        mit d = halbe Spurweite, r = Radradius.
        """
        forward = self._target_twist.linear.x
        angular = self._target_twist.angular.z

        if self._is_cmd_timed_out():
            forward = 0.0
            angular = 0.0
            self._target_twist = Twist()

        left_vel = (forward - angular * _HALF_WHEEL_SEPARATION) / _WHEEL_RADIUS
        right_vel = (forward + angular * _HALF_WHEEL_SEPARATION) / _WHEEL_RADIUS

        self._left_motor.setVelocity(left_vel)
        self._right_motor.setVelocity(right_vel)

    def _is_cmd_timed_out(self) -> bool:
        """Gibt True zurück, wenn seit dem letzten cmd_vel mehr als cmd_timeout vergangen ist.

        Returns:
            True bei Timeout oder fehlendem Zeitstempel.
        """
        if self._last_cmd_time is None:
            return True
        try:
            now = self._ros_node.get_clock().now()
            elapsed = (now.nanoseconds - self._last_cmd_time.nanoseconds) / 1e9
            return elapsed > self._cmd_timeout
        except Exception:  # pylint: disable=broad-except
            return True