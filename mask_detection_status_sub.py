import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
qos_profile = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

class MaskDetectionStatusSub(Node):
    def __init__(self):
        super().__init__('sub_detection_status')
        self.subscription = self.create_subscription(
            Bool,
            'mask_detection_status',
            self.callback_sub,
            qos_profile
        )

        self.detection_status = False

    def callback_sub(self, msg):
        if msg.data:
            self.detection_status = True
            self.get_logger().info("Mask detected, shutting down node.")
            raise RuntimeError