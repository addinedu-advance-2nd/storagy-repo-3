import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom', 
            self.listener_callback,
            10)
        self.subscription
        self.position_data = None  # 마지막으로 수신한 위치 데이터를 저장
        self.timer_period = 1.0  # 1초 주기
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def listener_callback(self, msg):
        # 오도메트리 메시지에서 위치와 방향 정보 추출
        self.position_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'orientation_z': msg.pose.pose.orientation.z,
            'orientation_w': msg.pose.pose.orientation.w
        }

    def timer_callback(self):
        if self.position_data is not None:
            # 로그 출력
            self.get_logger().info(f'Position: x={self.position_data["x"]}, y={self.position_data["y"]}, z={self.position_data["z"]}, Orientation: z={self.position_data["orientation_z"]}, w={self.position_data["orientation_w"]}')

def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()

    rclpy.spin(odometry_subscriber)
    odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
