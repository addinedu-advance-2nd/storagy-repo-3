import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge

bridge = CvBridge()
qos_profile = QoSProfile(
            depth = 10,
            history = HistoryPolicy.KEEP_LAST,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE
            )

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')

        self.publisher = self.create_publisher(
            CompressedImage,
            'compressed_image',
            qos_profile)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # OpenCV로 카메라 초기화 (인덱스 0번)
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')

            raise RuntimeError('Failed to open camera')
        
        self.get_logger().info('Compressed image publisher activated.')


    def timer_callback(self):
        # 카메라에서 프레임 읽기
        _, frame = self.cap.read()
        frame = cv2.resize(frame, (640, 480))
        img_msg = bridge.cv2_to_compressed_imgmsg(frame)
        self.publisher.publish(img_msg)


    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    compressed_image_publisher = CompressedImagePublisher()
    try:
        rclpy.spin(compressed_image_publisher)
    except KeyboardInterrupt:
        compressed_image_publisher.get_logger().info('Shutting down node.')
    finally:
        compressed_image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
