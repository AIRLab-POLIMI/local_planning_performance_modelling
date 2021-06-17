from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_benchmark_supervisor_node = Node(
        package='localization_performance_modelling',
        executable='localization_benchmark_supervisor',
        name='localization_benchmark_supervisor',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('configuration')],
    )
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        DeclareLaunchArgument(
            'configuration',
            description='Configuration yaml file path'),

        DeclareLaunchArgument(
            'use_sim_time',
            description='Use simulation/bag clock if true'),

        localization_benchmark_supervisor_node,

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'node_names': ['map_server',
                                        #'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'waypoint_follower']},
                        {'autostart': True}]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=localization_benchmark_supervisor_node,
                on_exit=EmitEvent(event=Shutdown(reason='supervisor_finished')))),
    ])
