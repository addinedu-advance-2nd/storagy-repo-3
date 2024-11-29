import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MessagePublisher(Node):
    def __init__(self):
        super().__init__('message_publisher')
        self.publisher = self.create_publisher(String, 'user_message', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 메시지 발행
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'발행된 메시지: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    message_publisher = MessagePublisher()
    rclpy.spin(message_publisher)
    message_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
