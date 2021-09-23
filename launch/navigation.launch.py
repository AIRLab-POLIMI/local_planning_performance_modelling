# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'map_server',
        'bt_navigator',
    ]

    return LaunchDescription([

        # Set env var to print messages to stdout immediately and with color
        SetEnvironmentVariable(
            name='RCUTILS_LOGGING_BUFFERED_STREAM',
            value='1'),
        SetEnvironmentVariable(
            name='RCUTILS_COLORIZED_OUTPUT',
            value='1'),

        DeclareLaunchArgument(
            'local_planner_params_file',
            description='Full path to the ROS2 parameters file to use for the local planner (controller server)'),
        DeclareLaunchArgument(
            'global_planner_params_file',
            description='Full path to the ROS2 parameters file to use for the global planner (planner server)'),
        DeclareLaunchArgument(
            'nav_params_file',
            description='Full path to the ROS2 parameters file to use for the rest of the navigation stack nodes'),

        # Local planner
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('local_planner_params_file'), LaunchConfiguration('nav_params_file')]),

        # Global planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('global_planner_params_file'), LaunchConfiguration('nav_params_file')]),

        # Rest of the navigation stack
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[LaunchConfiguration('nav_params_file')]),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[LaunchConfiguration('nav_params_file')]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('nav_params_file')]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gt_odom_static_transform_publisher',
            output='log',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'node_names': lifecycle_nodes},
                        {'autostart': True}]),

    ])
