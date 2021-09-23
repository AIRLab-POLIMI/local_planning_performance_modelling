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

    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['planner_server']

    return LaunchDescription([

        # Set env var to print messages to stdout immediately and with color
        SetEnvironmentVariable(
            name='RCUTILS_LOGGING_BUFFERED_STREAM',
            value='1'),
        SetEnvironmentVariable(
            name='RCUTILS_COLORIZED_OUTPUT',
            value='1'),

        DeclareLaunchArgument(
            'params_file',
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation_gp',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}]),

    ])
