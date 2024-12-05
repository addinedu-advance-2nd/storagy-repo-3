import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class OdometryController(Node):
    def __init__(self):
        super().__init__('odometry_controller')
        self.subscription = self.create_subscription(
            Odometry,
            'odom', 
            self.listener_callback,
            10)
        self.subscription
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 목표 위치 (x, y) 및 방향 (쿼터니언 z, w)
        self.goal_position = (0.2726, -4.5816)  # 목표 위치 (x, y)
        self.goal_orientation = (0.8209, 0.5709)  # 목표 방향 (쿼터니언 z, w)
        self.position_data = None
        self.timer_period = 1.0  # 1초 주기
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def listener_callback(self, msg):
        # 오도메트리 메시지에서 위치 및 방향 정보 추출
        self.position_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation_z': msg.pose.pose.orientation.z,
            'orientation_w': msg.pose.pose.orientation.w
        }

    def timer_callback(self):
        if self.position_data is not None:
            current_x = self.position_data['x']
            current_y = self.position_data['y']
            goal_x, goal_y = self.goal_position
            
            # 현재 위치와 목표 위치 간의 거리 계산
            distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
            self.get_logger().info(f'Current Position: x={current_x}, y={current_y}, Distance to Goal: {distance}')

            # 목표 위치에 도달했는지 확인
            if distance < 0.1:  # 목표 위치에 가까워지면
                self.get_logger().info('Goal reached!')
                self.stop_robot()
            else:
                self.move_towards_goal(current_x, current_y)

    def move_towards_goal(self, current_x, current_y):
        # 목표 방향 계산 (쿼터니언을 각도로 변환)
        goal_orientation_z, goal_orientation_w = self.goal_orientation
        goal_angle = self.quaternion_to_euler(goal_orientation_z, goal_orientation_w)
        
        # 현재 방향 계산 (쿼터니언을 각도로 변환)
        current_orientation = self.quaternion_to_euler(self.position_data['orientation_z'], self.position_data['orientation_w'])
        
        # 각도 차이 계산
        angle_diff = goal_angle - current_orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # -pi ~ pi 범위로 조정
        
        twist = Twist()
        
        # 로봇의 속도 설정
        if abs(angle_diff) > 0.1:  # 회전이 필요할 경우
            twist.linear.x = 0.0  # 전진 속도
            twist.angular.z = 0.5 if angle_diff > 0 else -0.5  # 회전 속도
        else:  # 목표 방향으로 전진
            twist.linear.x = 0.5  # 전진 속도
            twist.angular.z = 0.0  # 회전 속도
        
        self.publisher.publish(twist)

    def stop_robot(self):
        # 로봇 정지
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def quaternion_to_euler(self, z, w):
        # 쿼터니언을 각도로 변환
        siny_cosp = 2.0 * (w * z)
        cosy_cosp = 1.0 - 2.0 * (z * z)
        return math.atan2(siny_cosp, cosy_cosp)  # yaw 각도 반환

def main(args=None):
    rclpy.init(args=args)
    odometry_controller = OdometryController()

    rclpy.spin(odometry_controller)
    odometry_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
