from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

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
        self.get_logger().info(f"Received status: {msg.data}")
        self.status_queue.put(msg.data)