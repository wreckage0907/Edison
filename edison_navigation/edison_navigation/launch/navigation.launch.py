from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edison_navigation',
            executable='gps_simulator',
            name='gps_simulator',
            output='screen'
        ),
        Node(
            package='edison_navigation',
            executable='gps_handler',
            name='gps_handler',
            output='screen'
        ),
        Node(
            package='edison_navigation',
            executable='traffic_sign_detector',
            name='traffic_sign_detector',
            output='screen'
        ),
    ])