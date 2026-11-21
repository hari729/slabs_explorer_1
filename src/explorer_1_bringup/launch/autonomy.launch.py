import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    explorer_bringup_dir = get_package_share_directory('explorer_1_bringup')

    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller = LaunchConfiguration('controller')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            explorer_bringup_dir, 'config', 'slam_toolbox.yaml'),
        description='Full path to the SLAM Toolbox parameter file',
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            explorer_bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameter file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='mppi',
        choices=['mppi', 'rpp'],
        description='Local controller plugin to use: mppi or regulated_pure_pursuit',
    )

    controller_plugin = PythonExpression(
        ["'",
         controller,
         "' == 'rpp' and 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController' "
         "or 'nav2_mppi_controller::MPPIController'"])

    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'controller_server.ros__parameters.FollowPath.plugin': controller_plugin,
        },
        convert_types=True,
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'slam': 'False',
            'use_localization': 'False',
            'map': '',
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    return LaunchDescription([
        slam_params_file_arg,
        nav2_params_file_arg,
        use_sim_time_arg,
        controller_arg,
        slam_launch,
        nav2_launch,
    ])
