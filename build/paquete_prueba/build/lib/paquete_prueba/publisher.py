import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Accede a la cámara (0 es la cámara por defecto)

        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara.')
            return

        self.timer = self.create_timer(0.033, self.publish_image)  # 30 FPS

    def publish_image(self):
        ret, frame = self.cap.read()

        if ret:
            # Convertir la imagen a formato ROS
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando una imagen')

        else:
            self.get_logger().error('No se pudo capturar una imagen.')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)

    # Limpiar después de terminar
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
