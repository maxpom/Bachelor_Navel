"""Republiziert /lidar_left als Nav2-kompatibles /scan-Topic.

Stellt sicher, dass Zeitstempel streng monoton steigen, da Nav2-Komponenten
(insbesondere der Costmap-Layer) bei rückwärtslaufenden Zeitstempeln
TF-Extrapolationsfehler werfen.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSPresetProfiles, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRepublisher(Node):
    """Leitet /lidar_left nach /scan weiter mit monoton steigenden Zeitstempeln.

    Der Zeitstempel jeder ausgehenden Nachricht wird auf den aktuellen
    ROS2-Clock-Wert gesetzt. Falls der neue Stempel nicht streng größer als
    der vorherige ist (z.B. durch Simulationsartefakte), wird er um 1 ns
    erhöht, um Monotonizität zu garantieren.
    """

    def __init__(self):
        super().__init__('scan_republisher')

        # RELIABLE QoS für Nav2-Kompatibilität (Costmap erwartet verlässliche Lieferung).
        qos = QoSPresetProfiles.SENSOR_DATA.value
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(LaserScan, '/lidar_left', self._scan_cb, qos)
        self._pub = self.create_publisher(LaserScan, '/scan', qos)

        self._last_stamp = None  # Letzter publizierter Zeitstempel (sec, nanosec).

    # -----------------------------------------------------------------------
    # Callback
    # -----------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        """Setzt einen monoton steigenden Zeitstempel und republiziert den Scan.

        Args:
            msg: Eingehende LaserScan-Nachricht von /lidar_left.
        """
        stamp = self.get_clock().now().to_msg()
        stamp = self._ensure_monotonic(stamp)

        self._last_stamp = stamp
        msg.header.stamp = stamp

        if not msg.header.frame_id:
            msg.header.frame_id = 'lidar_left_link'

        self._pub.publish(msg)

    # -----------------------------------------------------------------------
    # Hilfsmethoden
    # -----------------------------------------------------------------------

    def _ensure_monotonic(self, stamp):
        """Stellt sicher, dass stamp streng größer als der letzte Stempel ist.

        Falls stamp <= _last_stamp, wird stamp um 1 ns erhöht und ein
        eventueller Überlauf der Nanosekunden behandelt.

        Args:
            stamp: Kandidaten-Zeitstempel (ROS2 Time Message).

        Returns:
            Zeitstempel mit garantierter Monotonizität.
        """
        if self._last_stamp is None:
            return stamp

        last = self._last_stamp
        is_not_strictly_greater = (
            stamp.sec < last.sec or
            (stamp.sec == last.sec and stamp.nanosec <= last.nanosec)
        )

        if is_not_strictly_greater:
            stamp.sec = int(last.sec)
            stamp.nanosec = int(last.nanosec) + 1
            if stamp.nanosec >= 10**9:
                stamp.sec += 1
                stamp.nanosec -= 10**9

        return stamp


def main(args=None):
    rclpy.init(args=args)
    node = ScanRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()