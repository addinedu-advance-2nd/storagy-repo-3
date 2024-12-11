import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavigationClient:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('navigation_client')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        
        # 목표 위치 설정
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z 
        goal_msg.pose.pose.orientation.w = w

        # 서버가 준비될 때까지 대기
        self.action_client.wait_for_server()

        # 목표 전송
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle:
            self.node.get_logger().error('Goal was rejected!')
            return
        
        self.node.get_logger().info('Goal accepted!')

        # 목표 상태 피드백 수신
        self.action_client.get_result_async(goal_handle).add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.node.get_logger().info('Navigation completed successfully!')
        else:
            self.node.get_logger().error('Navigation failed!')

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()
