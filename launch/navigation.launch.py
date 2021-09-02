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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['recoveries_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'map_server']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56


    # Create our own temporary YAML files that include substitutions
    # param_substitutions = {
    #     'default_bt_xml_filename': default_bt_xml_filename,
    #     'map_subscribe_transient_local': map_subscribe_transient_local}

    # configured_params = RewrittenYaml(
    #         source_file=params_file,
    #         param_rewrites=param_substitutions,
    #         convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        #SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        #SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),

        DeclareLaunchArgument(
            'params_file',
            description='Full path to the ROS2 parameters file to use'),

        # DeclareLaunchArgument(
        #     'default_bt_xml_filename',
        #     default_value=os.path.join(
        #         get_package_share_directory('nav2_bt_navigator'),
        #         'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        #     description='Full path to the behavior tree xml file to use'),

        # DeclareLaunchArgument(
        #     'map_subscribe_transient_local', default_value='false',
        #     description='Whether to set the map subscriber QoS to transient local'),
            

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file]),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file]),
            
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gt_odom_static_transform_publisher',
            output='log',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation_nav',
            output='screen',
            parameters=[{'node_names': lifecycle_nodes},
            {'autostart':True}]),

    ])
