import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    lidar_denoise_node = Node(
        package='utils', 
        executable='lidar_denoiser', 
        output='screen'
    )

    small_obstacle_detector_node = Node(
        package='utils', 
        executable='obstacle_detector', 
        output='screen'
    )

    compressed_image_publisher_node = Node(
    	package='utils', 
        executable='compressed_image_publisher', 
        output='screen'
    )

    tts_service_service_node = Node(
    	package='utils', 
        executable='tts_service', 
        output='screen'
    )

    aruco_marker_control_service_node = Node(
    	package='utils', 
        executable='aruco_marker_control_service', 
        output='screen'
    )


    # create and return launch description object
    return LaunchDescription(
        [
            lidar_denoise_node, 
            small_obstacle_detector_node, 
            compressed_image_publisher_node, 
            tts_service_service_node, 
            aruco_marker_control_service_node,
        ]
    )
