"""Bewegungserkennungs-Node auf Basis von Frame-Differenz (OpenCV).

Vergleicht jeden eingehenden Kamera-Frame mit einem gespeicherten
Referenz-Frame und publiziert ein Bool-Flag bei erkannter Bewegung.
Der Referenz-Frame wird über einen Service-Call zurückgesetzt und die
Erkennung dabei aktiviert.

Quelle Algorithmus:
  [MOTION_CV] Soka Coding (2021). Simple Motion Detection with Python and
              OpenCV for Beginners.
              https://sokacoding.medium.com/simple-motion-detection-with-
              python-and-opencv-for-beginners-cdd4579b2319
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class MotionDetector(Node):
    """Erkennt Bewegung durch Frame-Differenz gegenüber einem Referenz-Frame.

    Workflow:
      1. Node startet inaktiv (is_active=False).
      2. Service /reset_motion_detection aufrufen → setzt Referenz-Frame
         zurück und aktiviert Erkennung.
      3. Jeder folgende Frame wird mit dem ersten Frame nach Aktivierung
         verglichen. Konturenfläche > min_contour_area → Bewegung erkannt.

    Parameter:
      camera_topic       (str,   default: '/head_camera/image_color')
      detection_topic    (str,   default: '/pedestrian_visible')
      blur_kernel_size   (int,   default: 21)   — Gaussian-Blur-Kernelgröße.
      threshold_value    (int,   default: 30)   — Binärisierungsschwelle.
      min_contour_area   (float, default: 500)  — Mindestfläche in Pixeln.
    """

    def __init__(self):
        super().__init__('motion_detector')

        # --- Parameter ---
        self.declare_parameter('camera_topic',     '/head_camera/image_color')
        self.declare_parameter('detection_topic',  '/pedestrian_visible')
        self.declare_parameter('blur_kernel_size', 21)
        self.declare_parameter('threshold_value',  30)
        self.declare_parameter('min_contour_area', 500)

        camera_topic    = self.get_parameter('camera_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        self._blur_kernel  = self.get_parameter('blur_kernel_size').value
        self._threshold    = self.get_parameter('threshold_value').value
        self._min_area     = self.get_parameter('min_contour_area').value

        # --- Zustand ---
        self._bridge      = CvBridge()
        self._ref_frame   = None   # Referenz-Frame (Grayscale).
        self._is_active   = False
        self._frame_count = 0

        # --- ROS2-Kommunikation ---
        self.create_subscription(Image, camera_topic, self._image_cb, 10)
        self._detection_pub = self.create_publisher(Bool, detection_topic, 10)
        self.create_service(Trigger, '/reset_motion_detection', self._reset_cb)

    # -----------------------------------------------------------------------
    # Service-Callback
    # -----------------------------------------------------------------------

    def _reset_cb(self, request, response):
        """Setzt den Referenz-Frame zurück und aktiviert die Erkennung.

        Args:
            request:  Ungenutzte Trigger-Request.
            response: Trigger-Response.

        Returns:
            Befüllte Response mit success=True.
        """
        self._ref_frame   = None
        self._is_active   = True
        self._frame_count = 0

        self.get_logger().info('Referenz-Frame zurückgesetzt.')
        self.get_logger().info('Status: AKTIV')
        self.get_logger().info('=' * 70)

        response.success = True
        response.message = 'Motion detection aktiviert. Warte auf Referenz-Frame.'
        return response

    # -----------------------------------------------------------------------
    # Bild-Callback
    # -----------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        """Verarbeitet einen eingehenden Kamera-Frame.

        Pipeline [MOTION_CV]:
          1. ROS Image → OpenCV BGR
          2. Graustufen + Gaussian Blur
          3. Absolut-Differenz zum Referenz-Frame
          4. Binärisierung + Dilatation
          5. Konturenerkennung → Bewegungsflag publizieren

        Args:
            msg: Eingehende Image-Nachricht.
        """
        if not self._is_active:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray  = cv2.GaussianBlur(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                (self._blur_kernel, self._blur_kernel),
                0,
            )

            # Ersten Frame als Referenz speichern.
            if self._ref_frame is None:
                self._ref_frame = gray
                self.get_logger().info('Referenz-Frame initialisiert.')
                return

            motion_detected = self._detect_motion(gray)

            detection_msg = Bool()
            detection_msg.data = motion_detected
            self._detection_pub.publish(detection_msg)
            self._frame_count += 1

        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().error(f'Bildverarbeitungsfehler: {e}')

    # -----------------------------------------------------------------------
    # Hilfsmethoden
    # -----------------------------------------------------------------------

    def _detect_motion(self, gray) -> bool:
        """Vergleicht einen Graustufen-Frame mit dem Referenz-Frame.

        Args:
            gray: Vorverarbeiteter Graustufen-Frame (numpy array).

        Returns:
            True wenn mindestens eine Kontur ≥ min_contour_area gefunden.
        """
        delta  = cv2.absdiff(self._ref_frame, gray)
        thresh = cv2.threshold(delta, self._threshold, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)

        contours, _ = cv2.findContours(
            thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return any(cv2.contourArea(c) >= self._min_area for c in contours)


def main(args=None):
    rclpy.init(args=args)
    node = MotionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()