import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class AMCLLogger(Node):
    def __init__(self):
        super().__init__('amcl_logger')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.listener_callback,
            10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초 주기로 타이머 설정
        self.pose_data = None
        self.file = open('pose_data.txt', 'w')  # 파일 열기
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # 속도 명령 퍼블리셔
        self.target_position = (1.0, 1.0)  # 목표 위치 (x, y)
        
        self.get_last_pose()

    def listener_callback(self, msg):
        self.pose_data = msg.pose.pose

    def timer_callback(self):
        if self.pose_data:
            x = self.pose_data.position.x
            y = self.pose_data.position.y
            z = self.pose_data.orientation.z
            w = self.pose_data.orientation.w
            log_message = f'Position: x={x}, y={y}, Orientation: z={z}, w={w}\n'
            self.get_logger().info(log_message)
            self.file.write(log_message)  # 파일에 데이터 기록
            self.file.flush()  # 버퍼 비우기
        else:
            self.get_logger().info('No pose data received yet.')

    def get_last_pose(self):
        try:
            with open('pose_data.txt', 'r') as file:
                lines = file.readlines()
                if lines:
                    last_line = lines[-1]
                    self.get_logger().info(f'Last pose data: {last_line.strip()}')
                    # 마지막 포즈 데이터 파싱
                    last_position = last_line.split(',')[0].split('=')
                    x = float(last_position[1])
                    y = float(last_position[2].split(' ')[0])
                    self.move_to_target(x, y)
        except FileNotFoundError:
            self.get_logger().error('pose_data.txt not found.')

    def move_to_target(self, current_x, current_y):
        target_x, target_y = self.target_position
        twist = Twist()

        # 간단한 이동 로직
        if current_x < target_x:
            twist.linear.x = 0.1  # 전진
        elif current_x > target_x:
            twist.linear.x = -0.1  # 후진
        
        if current_y < target_y:
            twist.linear.y = 0.1  # 좌회전
        elif current_y > target_y:
            twist.linear.y = -0.1  # 우회전

        self.cmd_vel_pub.publish(twist)  # 속도 명령 발행

    def destroy_node(self):
        self.file.close()  # 노드 종료 시 파일 닫기
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    amcl_logger = AMCLLogger()

    rclpy.spin(amcl_logger)

    amcl_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
