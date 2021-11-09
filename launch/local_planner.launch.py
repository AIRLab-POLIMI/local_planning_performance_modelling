from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['controller_server']

    return LaunchDescription([

        # Set env var to print messages to stdout immediately and with color
        SetEnvironmentVariable(
            name='RCUTILS_LOGGING_BUFFERED_STREAM',
            value='1'),
        SetEnvironmentVariable(
            name='RCUTILS_COLORIZED_OUTPUT',
            value='1'),

        DeclareLaunchArgument(
            'log_path',
            description='Log path for this run'),
        SetEnvironmentVariable(
            name='ROS_LOG_DIR',
            value=LaunchConfiguration('log_path')),

        DeclareLaunchArgument(
            'params_file',
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='both',
            parameters=[params_file]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation_lp',
            output='both',
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}]),

    ])