"""Webots-Treiber für Fußgänger-Roboter (person_1 – person_N).

Eine einzige Klasse für alle Fußgänger-Instanzen. Der Roboter-Name
wird über das <robot_name>-Property in der URDF übergeben und bestimmt
die Topic-Präfixe für cmd_vel und joint_states.

URDF-Beispiel (pedestrian2.urdf):
  <plugin type="my_package.pedestrian_driver.PedestrianDriver">
    <robot_name>person_2</robot_name>
  </plugin>
"""

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# Kinematik-Konstanten (Pioneer 3-DX).
_HALF_WHEEL_SEPARATION = 0.165  # m  (Spurweite 0.33 m / 2)
_WHEEL_RADIUS = 0.0975           # m


class PedestrianDriver:
    """Webots-Plugin-Klasse für einen namespaced Fußgänger-Roboter.

    Die Methoden `init` und `step` werden von webots_ros2_driver aufgerufen.
    Der robot_name aus den URDF-Properties bestimmt alle Topic-Namen.
    """

    def init(self, webots_node, properties):
        """Initialisiert Motoren, Sensoren und ROS2-Kommunikation.

        Args:
            webots_node: WebotsController-Instanz (Webots Robot + ROS2 Node).
            properties:  Plugin-Properties aus der URDF; erwartet 'robot_name'.
        """
        self._robot = webots_node.robot
        self._ros_node = self._resolve_ros_node(webots_node)

        # robot_name aus URDF-Properties lesen — bestimmt alle Topic-Namen.
        self._robot_name = properties.get('robot_name', 'person_1')

        self._left_motor = self._robot.getDevice('left wheel')
        self._right_motor = self._robot.getDevice('right wheel')

        if not self._left_motor or not self._right_motor:
            print(f'[PedestrianDriver/{self._robot_name}] ERROR: Radmotoren nicht gefunden!')
            return

        for motor in (self._left_motor, self._right_motor):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        timestep = int(self._robot.getBasicTimeStep())
        self._left_sensor  = self._enable_sensor('left wheel sensor',  timestep)
        self._right_sensor = self._enable_sensor('right wheel sensor', timestep)

        self._target_twist  = Twist()
        self._cmd_timeout   = 0.5
        self._last_cmd_time = None

        ns = self._robot_name
        self._ros_node.create_subscription(
            Twist, f'{ns}/cmd_vel', self._cmd_vel_callback, 1)
        self._ros_node.create_subscription(
            Twist, f'/{ns}/cmd_vel', self._cmd_vel_callback, 1)

        self._joint_state_pub = self._ros_node.create_publisher(
            JointState, f'{ns}/joint_states', 10)

        print(f'[PedestrianDriver/{ns}] Initialisierung abgeschlossen.')

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
        node = rclpy.create_node('pedestrian_driver')
        print('[PedestrianDriver] Fallback: eigener ROS2-Node erstellt.')
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
            twist: Eingehende Twist-Nachricht.
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
            return
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
        """Berechnet Radgeschwindigkeiten und schreibt sie auf die Motoren."""
        forward = self._target_twist.linear.x
        angular = self._target_twist.angular.z

        if self._is_cmd_timed_out():
            forward = 0.0
            angular = 0.0
            self._target_twist = Twist()

        self._left_motor.setVelocity(
            (forward - angular * _HALF_WHEEL_SEPARATION) / _WHEEL_RADIUS)
        self._right_motor.setVelocity(
            (forward + angular * _HALF_WHEEL_SEPARATION) / _WHEEL_RADIUS)

    def _is_cmd_timed_out(self) -> bool:
        """Gibt True zurück wenn seit dem letzten cmd_vel mehr als cmd_timeout vergangen ist.

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