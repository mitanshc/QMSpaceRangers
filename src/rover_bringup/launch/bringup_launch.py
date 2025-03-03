from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo

def generate_launch_description():
    return LaunchDescription([
        Node(package='rover_control', executable='control_node', output='screen'),
        Node(package='rover_sensors', executable='imu_node', output='screen'),
        Node(package='rover_teleop', executable='rover_teleop', output='screen'),
    ])
