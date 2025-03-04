from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use simulation time'),
    DeclareLaunchArgument('robot_name', default_value='tcebot',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('launch_rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Launch RViz'),
]


def generate_launch_description():
    pkg_tcebot_description = get_package_share_directory('tcebot_description')
    xacro_file = PathJoinSubstitution([pkg_tcebot_description, 'urdf', 'tcebot.urdf.xacro'])
    rviz_config_file = PathJoinSubstitution([pkg_tcebot_description, 'rviz', 'config.rviz'])
    
    namespace = LaunchConfiguration('namespace')
    launch_rviz = LaunchConfiguration('launch_rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command(['xacro', ' ', xacro_file, ' ', 'namespace:=', namespace])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        condition=IfCondition(launch_rviz),  # Launch RViz only if 'launch_rviz' is true
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz2)

    return ld
