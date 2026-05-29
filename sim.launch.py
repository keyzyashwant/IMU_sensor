from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Was: imu_publisher → now: imu_serial_node
        Node(package='imu_sim', executable='imu_serial_node', name='imu_serial_node'),
        Node(package='imu_sim', executable='kalman_node',     name='kalman_node'),
    ])