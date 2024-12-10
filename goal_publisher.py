import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info('GoalPublisher 노드가 시작되었습니다.')

    def publish_goal(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        # goal_msg.pose.orientation.w = theta  # 필요시 방향 설정
        
        self.publisher_.publish(goal_msg)
        self.get_logger().info(f'Goal published: x={x}, y={y}')
    
def main(args=None):
    rclpy.init(args=args)

    node = GoalPublisher()
    time.sleep(2)  # 노드 초기화 대기

    # 목표 위치 설정 (예: x=1.0, y=2.0, 회전=0도)
    node.publish_goal(0.476, 0.76)

    rclpy.spin_once(node, timeout_sec=2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
