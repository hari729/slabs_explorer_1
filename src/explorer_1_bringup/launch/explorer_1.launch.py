from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_description_path = get_package_share_path('explorer_1_description')
    robot_bringup_path = get_package_share_path('explorer_1_bringup')

    urdf_path = os.path.join(robot_description_path, 'urdf', 'explorer_1.urdf.xacro')
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'explorer_1_controllers.yaml')
    ydlidar_config_path = os.path.join(
        get_package_share_path('ydlidar_ros2_driver'),
        'params',
        'X2.yaml',
    )

    use_esp32_hardware_arg = DeclareLaunchArgument(
        'use_esp32_hardware',
        default_value='true',
        description='Use real ESP32 hardware for the diff drive',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_esp32_hardware:=', LaunchConfiguration('use_esp32_hardware'),
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_node",
        parameters=[os.path.join(robot_bringup_path, "config", "ekf.yaml")],
        remappings=[
            ("/imu/data_raw", "/imu_sensor_broadcaster/imu"),
            ("/imu/data", "/imu/data"),
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(robot_bringup_path, "config", "ekf.yaml")],
    )

    ydlidar_node = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        parameters=[ydlidar_config_path],
        output="screen",
    )

    return LaunchDescription([
        use_esp32_hardware_arg,
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        imu_sensor_broadcaster_spawner,
        imu_filter_node,
        ekf_node,
        ydlidar_node,
    ])
