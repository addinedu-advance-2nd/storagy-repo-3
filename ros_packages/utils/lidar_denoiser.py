import time
import numpy as np
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from laser_geometry import LaserProjection
from sklearn.cluster import DBSCAN
class Denoiser(Node):
    def __init__(self):
        super().__init__('lidar_denoiser')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_v2',
            10
        )

        self.laser_proj = LaserProjection()

        self.get_logger().info('LiDAR denoiser activated.')


    def scan_callback(self, msg_sub):
        ts = time.time()

        scan = np.array(msg_sub.ranges)
        angles = np.linspace(msg_sub.angle_min, msg_sub.angle_max, scan.shape[0])
        points = np.array([scan*np.cos(angles), scan*np.sin(angles)]).T

        roi_idx = np.where(scan<=1.5)
        roi_points = points[roi_idx]

        threshold = 0.03
        clusterer = DBSCAN(eps=threshold, min_samples=2)
        clusterer.fit(roi_points)
        label = clusterer.labels_

        range_max = msg_sub.range_max
        roi_points[label==-1] = np.array([range_max, range_max])/np.sqrt(2)

        scan[roi_idx] = np.linalg.norm(roi_points, axis=1)
        intensities = np.array(msg_sub.intensities)

        header = Header(
            frame_id = msg_sub.header.frame_id,
            stamp = self.get_clock().now().to_msg()
        )

        msg_pub = LaserScan(
            header = header,
            angle_min = msg_sub.angle_min,
            angle_max = msg_sub.angle_max,
            angle_increment = msg_sub.angle_increment,
            time_increment = msg_sub.time_increment,
            scan_time = msg_sub.scan_time,
            range_min = msg_sub.range_min,
            range_max = msg_sub.range_max,
            ranges = scan.astype(np.float32).tobytes(),
            intensities = intensities.astype(np.float32).tobytes()
        )
        
        self.publisher.publish(msg_pub)
        # print(f'[LiDAR Denoiser] Runtime : {time.time()-ts:.3f} sec')


def main():
    rp.init()
    node = Denoiser()
    
    rp.spin(node)
    
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
