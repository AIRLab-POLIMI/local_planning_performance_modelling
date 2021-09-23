from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_planning_benchmark_supervisor_node = Node(
        package='local_planning_performance_modelling',
        executable='local_planning_benchmark_supervisor',
        name='local_planning_benchmark_supervisor',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('configuration')],
    )

    return LaunchDescription([

        # Set env var to print messages to stdout immediately and with color
        SetEnvironmentVariable(
            name='RCUTILS_LOGGING_BUFFERED_STREAM',
            value='1'),
        SetEnvironmentVariable(
            name='RCUTILS_COLORIZED_OUTPUT',
            value='1'),

        DeclareLaunchArgument(
            'configuration',
            description='Configuration yaml file path'),

        local_planning_benchmark_supervisor_node,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=local_planning_benchmark_supervisor_node,
                on_exit=EmitEvent(event=Shutdown(reason='supervisor_finished')))),
    ])
