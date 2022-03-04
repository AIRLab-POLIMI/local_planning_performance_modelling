from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
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

        # Player command
        DeclareLaunchArgument(
            'ros2_bag_output_path',
            description='Full path of the created bag files'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('ros2_bag_output_path')], output='both'),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='/home/enrico/w/ros2_ws/src/local_planning_performance_modelling/config/component_configurations/rviz/ros2_bag_play.rviz',
            description='Full path to the RVIZ config file to use'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            output='log'),

    ])
