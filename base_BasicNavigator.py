from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import queue
import pygame
from std_srvs.srv import Trigger

class DetectionSubscriber(Node):
    def __init__(self, status_queue):
        super().__init__('detection_subscriber')
        qos_profile = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Bool,
            'mask_detection_status',
            self.callback_sub,
            qos_profile
        )
        self.status_queue = status_queue  

    def callback_sub(self, msg):
        while not self.status_queue.empty():
            self.status_queue.get_nowait()
        #self.get_logger().info(f"Received status: {msg.data}")
        self.status_queue.put(msg.data)


class NavigationNode(Node):
    def __init__(self, status_queue):
        super().__init__('navigation_node')
        self.status_queue = status_queue  
        self.navigator = BasicNavigator()
        self.navigation_status = 0
        self.navigating = False
        self.previous_status = False
        self.false_count = 0
        self.goal_pose = None  
        self.music_file = "registration.mp3"
        self.music_file2 = "lose.mp3"
        # self.navigator.waitUntilNav2Active()
        # self.set_initial_pose()
        self.play_music(self.music_file)
        self.timer = self.create_timer(0.5, self.navigate_to_goal)

    # def set_initial_pose(self):
    #     initial_pose = PoseStamped()
    #     initial_pose.header.frame_id = 'map'
    #     initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
    #     initial_pose.pose.position.x = 0.0
    #     initial_pose.pose.position.y = 0.0
    #     initial_pose.pose.orientation.z = 0.0
    #     initial_pose.pose.orientation.w = 1.0
    #     self.navigator.setInitialPose(initial_pose)

    def navigate_to_goal(self):
        print('sdlkfhsklflsjfdljsd')

        if not self.goal_pose:  
            return

        try:
            status = self.status_queue.get_nowait()
        except queue.Empty:
            status = self.previous_status

        if status != self.previous_status:
            if status and not self.navigating:
                self.get_logger().info("주행 시작")
                self.navigator.goToPose(self.goal_pose)  # 미리 저장된 goal_pose 사용
                self.navigating = True
                self.navigation_status = 1
            elif not status and self.navigating:
                self.get_logger().info("주행 중지")
                self.navigator.cancelTask()
                self.navigating = False
            self.previous_status = status

        if not status:
            self.false_count += 1
            if self.false_count >= 40:
                self.get_logger().info("안내 문구 시작")
                self.play_music(self.music_file2)
                self.false_count = 0
        else:
            self.false_count = 0

        if self.navigating and self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.navigation_status = 2
                self.get_logger().info("Goal succeeded!")

                self.timer.destroy()
                raise RuntimeError
            elif result == TaskResult.CANCELED:
                self.navigation_status = 3
                self.get_logger().info("Goal was canceled!")
            elif result == TaskResult.FAILED:
                self.navigation_status = 3
                self.get_logger().info("Goal failed!")
            self.navigating = False

    def create_goal_pose(self, x, y, z, w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w
        return goal_pose

    def play_music(self, file):
        freq = 16000    # sampling rate
        bitsize = -16   # signed 16 bit
        channels = 1    # mono
        buffer = 2048   # buffer size
        pygame.mixer.init(freq, bitsize, channels, buffer)
        pygame.mixer.music.load(file)
        pygame.mixer.music.play()
        clock = pygame.time.Clock()
        while pygame.mixer.music.get_busy():
            clock.tick(30)

class ROS2ServiceClient(Node):
    def __init__(self):
        super().__init__('ros2_service_client')
        self.client = self.create_client(Trigger, 'home')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def call_service(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            raise RuntimeError("Service call failed.")
        
# def main(args=None):
#     rclpy.init(args=args)
#     status_queue = queue.Queue(maxsize=3)  
#     subscriber_node = DetectionSubscriber(status_queue)
#     navigation_node = NavigationNode(status_queue)
    
#     goal_pose = navigation_node.create_goal_pose(3.8, 0.572, 0.0, 1.0)
#     navigation_node.goal_pose = goal_pose  
#     executor = MultiThreadedExecutor()
#     executor.add_node(subscriber_node)
#     executor.add_node(navigation_node)
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         subscriber_node.get_logger().info("Shutting down.")
#     finally:
#         executor.shutdown()
#         subscriber_node.destroy_node()
#         navigation_node.destroy_node()
#         rclpy.shutdown()
#         pygame.mixer.quit()

# if __name__ == '__main__':
#     main()
