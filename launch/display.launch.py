from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('tcebot_description')

    urdf_path = os.path.join(package_dir, 'urdf', 'tcebot.urdf.xacro')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'display.rviz') 

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }],
        output='screen'
    )

    joint_state_publisher_gui_node = Node(
       package="joint_state_publisher_gui",
       executable="joint_state_publisher_gui"
   )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path], 
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
