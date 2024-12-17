import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavigationClient:
    def __init__(self):
        self.node = rclpy.create_node('navigation_client')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.navigation_status = 0 # 0 : 주행 전 / 1 : 주행 중 / 2 : 주행 성공 / 3 : 주행 실패

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
        self.navigation_status = 1

        # 목표 상태 수신
        self.node.create_timer(0.5, lambda: self.check_result(goal_handle))

    def check_result(self, goal_handle):
        future = goal_handle.get_result_async()
        future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if self.navigation_status == 1:  # 한 번만 출력
            if result:
                self.node.get_logger().info('Navigation completed successfully!')
                self.navigation_status = 2
            else:
                self.node.get_logger().error('Navigation f1ailed!')
                self.navigation_status = 3
            
            # 네비게이션 클라이언트 종료
            self.destroy()

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigation_client = NavigationClient()

    navigation_client.send_goal(0.257, 0.572, 0.0, 1.0)

    # 주기적으로 spin()을 호출하여 콜백을 처리
    rclpy.spin(navigation_client.node)

if __name__ == '__main__':
    main()
