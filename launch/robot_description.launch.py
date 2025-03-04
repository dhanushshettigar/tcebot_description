import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# Get package share directories
pkg_tcebot_bringup = get_package_share_directory('tcebot_bringup')
pkg_tcebot_control = get_package_share_directory('tcebot_control')
pkg_tcebot_description = get_package_share_directory('tcebot_description')
pkg_mpu6050driver = get_package_share_directory('mpu6050driver')

def generate_launch_description():
    # Declare launch arguments
    camera_arg = DeclareLaunchArgument(
        'include_camera', default_value='True', description='Include camera launch.'
    )
    camera = LaunchConfiguration('include_camera')

    rplidar_arg = DeclareLaunchArgument(
        'include_rplidar', default_value='True', description='Include rplidar launch.'
    )
    rplidar = LaunchConfiguration('include_rplidar')

    # Include tcebot_description launch file
    include_tcebot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_description, 'launch', 'robot_description.launch.py'),
        ),
        launch_arguments={'rsp': 'True'}.items(),
    )

    # Include tcebot_control launch file
    include_tcebot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_control, 'launch', 'tcebot_control.launch.py'),
        )
    )

    # Include RPLiDAR launch file
    include_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_bringup, 'launch', 'rplidar.launch.py'),
        ),
        launch_arguments={"serial_port": '/dev/ttyUSB0'}.items(),
        condition=IfCondition(rplidar),
    )

    # Include camera launch file
    include_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_bringup, 'launch', 'camera.launch.py'),
        ),
        condition=IfCondition(camera),
    )

    # Start MPU6050 driver first
    include_mpu6050_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mpu6050driver, 'launch', 'mpu6050driver_launch.py'),
        )
    )

    # Delay robot control and sensors to ensure IMU is publishing
    tcebot_control_timer = TimerAction(period=5.0, actions=[include_tcebot_control])
    rplidar_timer = TimerAction(period=3.0, actions=[include_rplidar])
    camera_timer = TimerAction(period=3.0, actions=[include_camera])

    # Include robot_localization (EKF) to generate /odom
    ekf_config_path = os.path.join(pkg_tcebot_bringup, 'config', 'ekf.yaml')

    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    ekf_timer = TimerAction(period=7.0, actions=[ekf_localization])  # Start EKF after IMU is running

    return LaunchDescription([
        include_tcebot_description,
        include_mpu6050_driver,  # Start MPU6050 first
        tcebot_control_timer,
        camera_arg,
        camera_timer,
        rplidar_arg,
        rplidar_timer,
        ekf_timer,  # Start EKF after IMU is publishing
    ])
