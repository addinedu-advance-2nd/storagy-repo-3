import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MessageSubscriber(Node):
    def __init__(self):
        super().__init__('message_subscriber')
        self.subscription = self.create_subscription(
            String,
            'user_message',  # 구독할 토픽 이름
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'받은 메시지: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    message_subscriber = MessageSubscriber()
    rclpy.spin(message_subscriber)
    message_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
