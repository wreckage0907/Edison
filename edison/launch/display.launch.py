import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get package directory
    package_name = 'edison'
    pkg_share = get_package_share_directory(package_name)

    # Process URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'edison.urdf.xacro')
    
    # Debug print
    print(f"Looking for XACRO file at: {xacro_file}")
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"XACRO file not found at {xacro_file}")
    
    # Process the XACRO file
    robot_description_content = Command([
        'xacro ', xacro_file, ' sim_mode:=true'
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])