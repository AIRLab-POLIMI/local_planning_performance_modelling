from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_model_path_env_var',
            description='GAZEBO_MODEL_PATH environment variable'),
        DeclareLaunchArgument(
            'gazebo_plugin_path_env_var',
            description='GAZEBO_PLUGIN_PATH environment variable'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Whether to execute gzclient'),
        DeclareLaunchArgument(
            'world_model_file',
            description='Full path to world model file to load'),
        #DeclareLaunchArgument(
            #'robot_gt_urdf_file',
            #description='Full path to robot urdf model file to load'),
        DeclareLaunchArgument(
            'robot_realistic_urdf_file',
            description='Full path to robot urdf model file to load'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', LaunchConfiguration('gazebo_model_path_env_var')),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', LaunchConfiguration('gazebo_plugin_path_env_var')),
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world_model_file')],
            output='screen'),
        ExecuteProcess(
            condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('headless')])),
            cmd=['gzclient'],
            output='log'),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher_gt',
        #     output='log',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=[LaunchConfiguration('robot_gt_urdf_file')]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gt_odom_static_transform_publisher',
            output='log',
            parameters=[{'use_sim_time': True}],
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),  
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_realistic',
            output='log',
            parameters=[{'use_sim_time': True}],
            arguments=[LaunchConfiguration('robot_realistic_urdf_file')]),
    ])

    return ld
