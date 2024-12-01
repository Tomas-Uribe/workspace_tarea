import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.process_image,
            10)
        self.publisher_ = self.create_publisher(Image, 'processed_image', 10)
        self.bridge = CvBridge()

        # Cargar templates
        self.template1 = cv2.imread('/templateCara.jpg', cv2.IMREAD_GRAYSCALE)
        self.template2 = cv2.imread('/templateCara2.jpg', cv2.IMREAD_GRAYSCALE)

    def process_image(self, msg):
        try:
            # Convertir la imagen de ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir la imagen: {e}')
            return

        # Convertir la imagen a escala de grises
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Realizar template matching con el primer template
        result = cv2.matchTemplate(gray_image, self.template1, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

        # Dibujar un rectángulo donde se encontró la coincidencia máxima
        top_left = max_loc
        bottom_right = (top_left[0] + self.template1.shape[1], top_left[1] + self.template1.shape[0])
        cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)

        # Convertir la imagen procesada a mensaje ROS y publicarla
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
