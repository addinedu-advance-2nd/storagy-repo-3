'''
Tr_depth_to_footprint
array([[ 0.    ,  0.    ,  1.    ,  0.198 ],
       [-1.    ,  0.    ,  0.    ,  0.    ],
       [ 0.    , -1.    ,  0.    ,  0.2945],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
'''

import sys
import time

import ros2_numpy as rnp
import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import HDBSCAN

import rclpy as rp
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from laser_geometry import LaserProjection


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('low_object_detector')

        self.depth_pcd_subscriber = Subscriber(
            self, 
            PointCloud2, 
            '/camera/depth/points'
        )

        self.laser_proj = LaserProjection()
        self.lidar_scan_subscriber = Subscriber(
            self, 
            LaserScan,
            '/scan' 
        )

        self.synchronizer = ApproximateTimeSynchronizer(
            [self.depth_pcd_subscriber, self.lidar_scan_subscriber], 
            queue_size=5, 
            slop=0.02
        )        
        self.synchronizer.registerCallback(self.detect_object)

        self.publisher = self.create_publisher(
            PointCloud2, 
            '/obstacle_points', 
            10
        )
        self.get_logger().info('Small obstacle detector activated.')

    def detect_object(self, depth_pcd_msg, lidar_scan_msg):
        ts = time.time()
        # Get points (Nx3)
        depth_points = rnp.numpify(depth_pcd_msg)['xyz']
        
        lidar_pcd_msg = self.laser_proj.projectLaser(lidar_scan_msg)
        lidar_points = rnp.numpify(lidar_pcd_msg)['xyz']

        # Transform
        depth_points = np.array([1, -1, -1])*depth_points[:, np.array([2, 0, 1])]+np.array([0.198, 0, 0.2945])
        lidar_points = np.array([1, -1, -1])*lidar_points+np.array([0.1525, 0, 0.127])
        
        # Remove points out of ROI
        depth_points = self.remove_points_out_of_ROI(depth_points, (0, 3), (-1, 1), (0.004, 0.13))

        lidar_points = self.remove_points_out_of_ROI(lidar_points, (0, 3), (-1, 1), (-1, 1))
        rounded_depth_points = self.round_points(depth_points)

        # Remove points near the walls
        kd_tree_depth = cKDTree(rounded_depth_points[:,:2])
        kd_tree_lidar = cKDTree(lidar_points[:,:2])

        p = kd_tree_lidar.query_ball_tree(kd_tree_depth, r=0.05)
        pairs = np.int_(np.concatenate(p))
        idx = np.zeros((rounded_depth_points.shape[0], ), dtype=np.bool_)
        idx[pairs] = True

        object_points = rounded_depth_points[~idx]

        # extract obstacle points
        obstacle_points = np.zeros((0, 3))
        if object_points.shape[0]>=20:
            # clustering
            clusterer = HDBSCAN(min_cluster_size=10)
            clusterer.fit(object_points)
            labels = clusterer.labels_

            c_idx = 0
            while True:
                points = object_points[labels==c_idx]
                if points.shape[0] == 0:
                    break
                    
                if points.shape[0] >= 20:
                    size = 100*(np.max(points, axis=0)-np.min(points, axis=0))
                    volume = size[0]*size[1]*size[2]
                    if volume >= 4.0:
                        # Consider point clusters as obstacle
                        obstacle_points = np.concatenate([obstacle_points, points], axis=0)
                c_idx += 1
        
        # Cretae topic message and publish
        msg = self.create_pointcloud2_msg(obstacle_points, stamp=lidar_scan_msg.header.stamp)
        self.publisher.publish(msg)
        print(f'[Small Object Detector] Runtime : {time.time()-ts:.3f} sec')

    def remove_points_out_of_ROI(self, points, x_range, y_range, z_range):
        mask = np.where(
            (x_range[0] <= points[:,0]) & (points[:,0] <= x_range[1]) &
            (y_range[0] <= points[:,1]) & (points[:,1] <= y_range[1]) &
            (z_range[0] <= points[:,2]) & (points[:,2] <= z_range[1])
        )
        return points[mask]
    
    def round_points(self, points):
        rounded_points = np.round(points, 2)
        result = np.unique(rounded_points, axis=0)
        return result
    
    def create_pointcloud2_msg(self, points, frame_id='base_footprint', stamp=None):
        header = Header(
            frame_id = frame_id, 
            stamp = self.get_clock().now().to_msg() if stamp is None else stamp
        )

        fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1), 
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1), 
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
        
        N = points.shape[0]
        data = points.astype(np.float32).tobytes()
        itemsize = np.dtype(np.float32).itemsize
        
        return PointCloud2(
            header = header, 
            height = 1, 
            width = N,
            is_dense = False,
            is_bigendian = False,
            fields = fields, 
            point_step = itemsize*len(fields), 
            row_step = (itemsize*len(fields))*N, 
            data = data
        )
    
def main(args=None):
    rp.init(args=args)

    detector_node = ObjectDetectorNode()
    # print('Ready to detect')

    rp.spin(detector_node)

    detector_node.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()

