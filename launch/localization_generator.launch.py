from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

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
            'localization_params_file',
            description='Full path to the ROS2 parameters file to use for the localization nodes'),

        # Localization
        Node(
            package='local_planning_performance_modelling',
            executable='localization_generator',
            name='localization_generator',
            output='both',
            parameters=[LaunchConfiguration('localization_params_file')]),

    ])
