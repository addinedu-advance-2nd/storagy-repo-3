import os
import glob
from setuptools import find_packages, setup

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='storagy',
    maintainer_email='storagy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'lidar_denoiser = utils.lidar_denoiser:main',
        'obstacle_detector = utils.obstacle_detector:main',
        'compressed_image_publisher = utils.compressed_image_publisher:main',
        'tts_service = utils.tts_service:main',
        'aruco_marker_control_service = utils.aruco_marker_control_service:main',
        ],
    },
)
