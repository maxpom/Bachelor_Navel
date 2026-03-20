"""Beispiel-Node: publiziert synthetische JointStates und odom→axis TF.

Demonstriert die Verwendung von TransformBroadcaster und JointState-
Publisher mit einem einfachen kinematischen Modell (Kreisbahn + Kippbewegung).

Hinweis: Dieser Node dient ausschließlich zu Demonstrationszwecken und
wird im produktiven Simulationsstack nicht verwendet.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):
    """Publiziert animierte JointStates und eine odom→axis TF-Transformation.

    Gelenke:
      swivel    — kontinuierliche Rotation um z (1°/Schritt).
      tilt      — Pendelbewegung im Bereich [-0.5, 0.0] rad.
      periscope — Hubbewegung im Bereich [0.0, 0.2] m.

    Der Roboter bewegt sich auf einem Kreisbogen mit Radius 2 m.
    """

    _DEGREE = math.pi / 180.0  # 1° in Radiant.
    _RATE_HZ = 30.0

    def __init__(self):
        super().__init__('state_publisher')

        qos = QoSProfile(depth=10)
        self._joint_pub  = self.create_publisher(JointState, 'joint_states', qos)
        self._broadcaster = TransformBroadcaster(self, qos=qos)

        # Gelenkzustand.
        self._tilt   = 0.0
        self._tinc   = self._DEGREE
        self._swivel = 0.0
        self._height = 0.0
        self._hinc   = 0.005
        self._angle  = 0.0

        # TF-Nachricht vorbereiten (Header wird pro Schritt aktualisiert).
        self._odom_trans = TransformStamped()
        self._odom_trans.header.frame_id = 'odom'
        self._odom_trans.child_frame_id  = 'axis'

        self.create_timer(1.0 / self._RATE_HZ, self._timer_cb)
        self.get_logger().info(f'{self.get_name()} gestartet.')

    # -----------------------------------------------------------------------
    # Timer-Callback
    # -----------------------------------------------------------------------

    def _timer_cb(self):
        """Aktualisiert Gelenkzustand und TF-Transformation (30 Hz)."""
        now = self.get_clock().now().to_msg()

        # JointState publizieren.
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name         = ['swivel', 'tilt', 'periscope']
        joint_state.position     = [self._swivel, self._tilt, self._height]
        self._joint_pub.publish(joint_state)

        # TF odom → axis (Kreisbahn Radius 2 m).
        self._odom_trans.header.stamp = now
        self._odom_trans.transform.translation.x = math.cos(self._angle) * 2.0
        self._odom_trans.transform.translation.y = math.sin(self._angle) * 2.0
        self._odom_trans.transform.translation.z = 0.7
        self._odom_trans.transform.rotation = _euler_to_quaternion(
            0.0, 0.0, self._angle + math.pi / 2.0)
        self._broadcaster.sendTransform(self._odom_trans)

        # Zustand für nächsten Schritt aktualisieren.
        self._tilt += self._tinc
        if self._tilt < -0.5 or self._tilt > 0.0:
            self._tinc *= -1

        self._height += self._hinc
        if self._height > 0.2 or self._height < 0.0:
            self._hinc *= -1

        self._swivel += self._DEGREE
        self._angle  += self._DEGREE / 4.0


# ---------------------------------------------------------------------------
# Modul-Hilfsfunktion
# ---------------------------------------------------------------------------


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Konvertiert Euler-Winkel (RPY) in ein Quaternion.

    Verwendet die ZYX-Konvention (yaw → pitch → roll).

    Args:
        roll:  Rotation um x-Achse in rad.
        pitch: Rotation um y-Achse in rad.
        yaw:   Rotation um z-Achse in rad.

    Returns:
        Quaternion-Nachricht.
    """
    sr, cr = math.sin(roll  / 2.0), math.cos(roll  / 2.0)
    sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    sy, cy = math.sin(yaw   / 2.0), math.cos(yaw   / 2.0)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()