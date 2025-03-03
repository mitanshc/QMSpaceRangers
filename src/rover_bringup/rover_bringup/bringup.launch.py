from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rover_control', executable='control_node', output='screen'),
        Node(package='rover_sensors', executable='imu_node', output='screen'),
        Node(package='rover_teleop', executable='rover_teleop_node', output='screen'),
    ])
