from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
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

        # Localization
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gt_odom_static_transform_publisher',
            output='log',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),

    ])
