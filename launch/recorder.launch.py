from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration


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
            'recorder_output_path',
            description='Full path of the created bag files'),

        # Recorder command
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--compression-mode', 'file', '--compression-format', 'zstd', '-a', '-o', LaunchConfiguration('recorder_output_path')], output='screen'),

    ])
