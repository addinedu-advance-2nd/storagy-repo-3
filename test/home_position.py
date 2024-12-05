import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import math
import json
import os

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.amcl_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10)
        self.current_pose = None
        self.last_pose = self.load_last_pose()  # 마지막 위치 정보를 파일에서 불러옴

        # 1초 주기로 현재 위치를 로그로 출력하는 타이머 설정
        self.create_timer(1.0, self.log_current_pose)

    def load_last_pose(self):
        """파일에서 마지막 위치 정보를 로드하는 함수"""
        if os.path.exists('last_pose.json'):
            with open('last_pose.json', 'r') as f:
                data = json.load(f)
                pose = PoseStamped()
                pose.pose.position.x = data['position_x']
                pose.pose.position.y = data['position_y']
                pose.pose.orientation.z = data['orientation_z']
                pose.pose.orientation.w = data['orientation_w']
                return pose
        return None

    def save_last_pose(self):
        """마지막 위치 정보를 파일에 저장하는 함수"""
        if self.current_pose is not None:
            data = {
                'position_x': self.current_pose.pose.position.x,
                'position_y': self.current_pose.pose.position.y,
                'orientation_z': self.current_pose.pose.orientation.z,
                'orientation_w': self.current_pose.pose.orientation.w
            }
            with open('last_pose.json', 'w') as f:
                json.dump(data, f)

    def amcl_pose_callback(self, msg):
        self.current_pose = msg
        self.last_pose = msg  # 마지막 위치 정보를 업데이트
        self.save_last_pose()  # 새로운 위치 정보를 파일에 저장

    def log_current_pose(self):
        # 현재 위치 또는 마지막 위치 정보를 로그로 출력
        pose_to_log = self.current_pose if self.current_pose is not None else self.last_pose
        if pose_to_log is not None:
            position_x = pose_to_log.pose.position.x
            position_y = pose_to_log.pose.position.y
            orientation_z = pose_to_log.pose.orientation.z
            orientation_w = pose_to_log.pose.orientation.w
            self.get_logger().info(f"Current Position -> x: {position_x}, y: {position_y}, Orientation -> z: {orientation_z}, w: {orientation_w}")
        else:
            self.get_logger().warn("No pose information available.")

    def move_to_goal(self, goal_x, goal_y, goal_orientation_z, goal_orientation_w):
        # 현재 위치가 없으면 마지막 위치 사용
        if self.current_pose is None and self.last_pose is not None:
            self.current_pose = self.last_pose

        if self.current_pose is None:
            self.get_logger().warn("Current pose not available.")
            return
        
        # 목표 위치와 현재 위치의 거리 계산
        dx = goal_x - self.current_pose.pose.position.x
        dy = goal_y - self.current_pose.pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # 목표 방향 각도 계산
        target_angle = math.atan2(dy, dx)

        # 로봇이 목표 방향으로 회전
        self.rotate_to_goal(target_angle)

        # 목표 위치로 이동
        self.move_forward(distance)

        # 목표 방향으로 회전 (목표 쿼터니언에서 yaw 각도 계산)
        target_yaw = self.get_yaw_from_quaternion(goal_orientation_z, goal_orientation_w)
        self.rotate_to_goal(target_yaw)

    def rotate_to_goal(self, target_angle):
        twist = Twist()
        angle_tolerance = 0.1
        rate = self.create_rate(10)  # 10Hz

        while True:
            if self.current_pose is None:
                continue

            current_angle = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)

            # 각도 차이 계산
            angle_diff = target_angle - current_angle

            # 각도 보정
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # 회전 속도 설정
            if abs(angle_diff) > angle_tolerance:
                twist.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                twist.angular.z = 0.0
                break

            self.publisher.publish(twist)
            rate.sleep()

    def move_forward(self, distance):
        twist = Twist()
        linear_speed = 0.2  # 이동 속도
        move_time = distance / linear_speed
        rate = self.create_rate(10)  # 10Hz

        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).seconds < move_time:
            twist.linear.x = linear_speed
            self.publisher.publish(twist)
            rate.sleep()

        # 정지
        twist.linear.x = 0.0
        self.publisher.publish(twist)

    def get_yaw_from_quaternion(self, z, w):
        """쿼터니언에서 yaw 각도를 반환하는 함수"""
        return math.atan2(2.0 * (z * w), 1.0 - 2.0 * (z ** 2))

def main(args=None):
    rclpy.init(args=args)
    mover = RobotMover()

    # 목표 위치 및 방향 설정 (예: x=1.0, y=1.0, z=0.0, w=1.0)
    mover.move_to_goal(0.5000568337951672, -0.03991187986429263, 0.5256651305502541, 0.8506915836680086)

    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
