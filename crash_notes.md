# BT navigator seg fault and rclpy/tf2 SystemError
```
benchmark: starting run 103
	environment_folder: /home/enrico/ds/performance_modelling/test_datasets/dataset/7A-2/
	parameters_combination_dict:
		beta: [0.0175, 0.0175, 0.005, 0.005]
		laser_scan_max_range: 3.5
		laser_scan_fov_deg: 90
t: 1591822175.8749077, run: 103, event: waiting_supervisor_finish
[INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-10-22-49-35-874394-enrico-zenbook-26283
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [localization_benchmark_supervisor-14]: process started with pid [26371]
[INFO] [rviz2-1]: process started with pid [26289]
[INFO] [gzserver-2]: process started with pid [26290]
[INFO] [gzclient-3]: process started with pid [26291]
[INFO] [robot_state_publisher-4]: process started with pid [26292]
[INFO] [static_transform_publisher-5]: process started with pid [26293]
[INFO] [robot_state_publisher-6]: process started with pid [26294]
[INFO] [controller_server-7]: process started with pid [26303]
[INFO] [planner_server-8]: process started with pid [26308]
[INFO] [recoveries_server-9]: process started with pid [26316]
[INFO] [bt_navigator-10]: process started with pid [26318]
[INFO] [waypoint_follower-11]: process started with pid [26330]
[INFO] [map_server-12]: process started with pid [26337]
[INFO] [amcl-13]: process started with pid [26361]
[INFO] [lifecycle_manager-15]: process started with pid [26382]
[robot_state_publisher-4] Initialize urdf model from file: /home/enrico/ds/performance_modelling/output/test_localization/run_103/components_configuration/gazebo/robot_gt.urdf
[robot_state_publisher-4] Parsing robot urdf xml string.
[robot_state_publisher-4] Link base_link had 7 children
[robot_state_publisher-4] Link camera_link_gt had 2 children
[robot_state_publisher-4] Link camera_depth_frame_gt had 1 children
[robot_state_publisher-4] Link camera_depth_optical_frame_gt had 0 children
[robot_state_publisher-4] Link camera_rgb_frame_gt had 1 children
[robot_state_publisher-4] Link camera_rgb_optical_frame_gt had 0 children
[robot_state_publisher-4] Link caster_back_left_link_gt had 0 children
[robot_state_publisher-4] Link caster_back_right_link_gt had 0 children
[robot_state_publisher-4] Link imu_link_gt had 0 children
[robot_state_publisher-4] Link base_scan_gt had 0 children
[robot_state_publisher-4] Link wheel_left_link_gt had 0 children
[robot_state_publisher-4] Link wheel_right_link_gt had 0 children
[robot_state_publisher-4] got segment base_footprint_gt
[robot_state_publisher-4] got segment base_link
[robot_state_publisher-4] got segment base_scan_gt
[robot_state_publisher-4] got segment camera_depth_frame_gt
[robot_state_publisher-4] got segment camera_depth_optical_frame_gt
[robot_state_publisher-4] got segment camera_link_gt
[robot_state_publisher-4] got segment camera_rgb_frame_gt
[robot_state_publisher-4] got segment camera_rgb_optical_frame_gt
[robot_state_publisher-4] got segment caster_back_left_link_gt
[robot_state_publisher-4] got segment caster_back_right_link_gt
[robot_state_publisher-4] got segment imu_link_gt
[robot_state_publisher-4] got segment wheel_left_link_gt
[robot_state_publisher-4] got segment wheel_right_link_gt
[robot_state_publisher-4] Adding fixed segment from base_footprint_gt to base_link
[robot_state_publisher-4] Adding fixed segment from base_link to camera_link_gt
[robot_state_publisher-4] Adding fixed segment from camera_link_gt to camera_depth_frame_gt
[robot_state_publisher-4] Adding fixed segment from camera_depth_frame_gt to camera_depth_optical_frame_gt
[robot_state_publisher-4] Adding fixed segment from camera_link_gt to camera_rgb_frame_gt
[robot_state_publisher-4] Adding fixed segment from camera_rgb_frame_gt to camera_rgb_optical_frame_gt
[robot_state_publisher-4] Adding fixed segment from base_link to caster_back_left_link_gt
[robot_state_publisher-4] Adding fixed segment from base_link to caster_back_right_link_gt
[robot_state_publisher-4] Adding fixed segment from base_link to imu_link_gt
[robot_state_publisher-4] Adding fixed segment from base_link to base_scan_gt
[robot_state_publisher-4] Adding moving segment from base_link to wheel_left_link_gt
[robot_state_publisher-4] Adding moving segment from base_link to wheel_right_link_gt
[robot_state_publisher-6] Initialize urdf model from file: /home/enrico/ds/performance_modelling/output/test_localization/run_103/components_configuration/gazebo/robot_realistic.urdf
[robot_state_publisher-6] Parsing robot urdf xml string.
[robot_state_publisher-6] Link base_link_realistic had 7 children
[robot_state_publisher-6] Link camera_link_realistic had 2 children
[robot_state_publisher-6] Link camera_depth_frame_realistic had 1 children
[robot_state_publisher-6] Link camera_depth_optical_frame_realistic had 0 children
[robot_state_publisher-6] Link camera_rgb_frame_realistic had 1 children
[robot_state_publisher-6] Link camera_rgb_optical_frame_realistic had 0 children
[robot_state_publisher-6] Link caster_back_left_link_realistic had 0 children
[robot_state_publisher-6] Link caster_back_right_link_realistic had 0 children
[robot_state_publisher-6] Link imu_link_realistic had 0 children
[robot_state_publisher-6] Link base_scan_realistic had 0 children
[robot_state_publisher-6] Link wheel_left_link_realistic had 0 children
[robot_state_publisher-6] Link wheel_right_link_realistic had 0 children
[robot_state_publisher-6] got segment base_footprint_realistic
[robot_state_publisher-6] got segment base_link_realistic
[robot_state_publisher-6] got segment base_scan_realistic
[robot_state_publisher-6] got segment camera_depth_frame_realistic
[robot_state_publisher-6] got segment camera_depth_optical_frame_realistic
[robot_state_publisher-6] got segment camera_link_realistic
[robot_state_publisher-6] got segment camera_rgb_frame_realistic
[robot_state_publisher-6] got segment camera_rgb_optical_frame_realistic
[robot_state_publisher-6] got segment caster_back_left_link_realistic
[robot_state_publisher-6] got segment caster_back_right_link_realistic
[robot_state_publisher-6] got segment imu_link_realistic
[robot_state_publisher-6] got segment wheel_left_link_realistic
[robot_state_publisher-6] got segment wheel_right_link_realistic
[robot_state_publisher-6] Adding fixed segment from base_footprint_realistic to base_link_realistic
[robot_state_publisher-6] Adding fixed segment from base_link_realistic to camera_link_realistic
[robot_state_publisher-6] Adding fixed segment from camera_link_realistic to camera_depth_frame_realistic
[robot_state_publisher-6] Adding fixed segment from camera_depth_frame_realistic to camera_depth_optical_frame_realistic
[robot_state_publisher-6] Adding fixed segment from camera_link_realistic to camera_rgb_frame_realistic
[robot_state_publisher-6] Adding fixed segment from camera_rgb_frame_realistic to camera_rgb_optical_frame_realistic
[robot_state_publisher-6] Adding fixed segment from base_link_realistic to caster_back_left_link_realistic
[robot_state_publisher-6] Adding fixed segment from base_link_realistic to caster_back_right_link_realistic
[robot_state_publisher-6] Adding fixed segment from base_link_realistic to imu_link_realistic
[robot_state_publisher-6] Adding fixed segment from base_link_realistic to base_scan_realistic
[robot_state_publisher-6] Adding moving segment from base_link_realistic to wheel_left_link_realistic
[robot_state_publisher-6] Adding moving segment from base_link_realistic to wheel_right_link_realistic
[amcl-13] [INFO] [amcl]: Creating
[map_server-12] [INFO] [map_server]: Creating
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Creating
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Creating and initializing lifecycle service clients
[gzserver-2] Gazebo multi-robot simulator, version 9.0.0
[gzserver-2] Copyright (C) 2012 Open Source Robotics Foundation.
[gzserver-2] Released under the Apache 2 License.
[gzserver-2] http://gazebosim.org
[gzserver-2] 
[localization_benchmark_supervisor-14] preparing to start run
[rviz2-1] [WARN] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-1] [WARN] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[gzserver-2] [Msg] Waiting for master.
[gzserver-2] [Msg] Connected to gazebo master @ http://127.0.0.1:11345
[gzserver-2] [Msg] Publicized address: 192.168.1.101
[gzserver-2] [INFO] [camera_driver]: Publishing camera info to [/camera/camera_info]
[gzserver-2] [INFO] [gazebo_ros_state]: Publishing states of gazebo models at [/model_states]
[gzserver-2] [INFO] [gazebo_ros_state]: Publishing states of gazebo links at [/link_states]
[gzserver-2] [INFO] [turtlebot3_diff_drive]: Wheel pair 1 separation set to [0.287000m]
[gzserver-2] [INFO] [turtlebot3_diff_drive]: Wheel pair 1 diameter set to [0.066000m]
[gzserver-2] [INFO] [turtlebot3_diff_drive]: Subscribed to [/cmd_vel]
[gzserver-2] [INFO] [turtlebot3_diff_drive]: Computing odometry with parametric error model
[gzserver-2] alpha1 [0.017500]
[gzserver-2] alpha2 [0.017500]
[gzserver-2] alpha3 [0.005000]
[gzserver-2] alpha4 [0.005000]
[gzserver-2] 
[gzserver-2] [INFO] [turtlebot3_diff_drive]: Advertise odometry on [/odom]
[gzserver-2] [INFO] [turtlebot3_diff_drive]: Publishing odom transforms between [odom_realistic] and [base_footprint_realistic]
[gzserver-2] [INFO] [turtlebot3_diff_drive]: Publishing ground truth odom transforms between [odom] and [base_footprint_gt]
[gzclient-3] [Err] [REST.cc:205] Error in REST request
[gzclient-3] 
[gzclient-3] libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
[gzserver-2] [INFO] [turtlebot3_joint_state]: Going to publish joint [wheel_left_joint]
[gzserver-2] [INFO] [turtlebot3_joint_state]: Going to publish joint [wheel_right_joint]
[localization_benchmark_supervisor-14] [WARN] [localization_benchmark_supervisor]: still waiting to receive first sensor message from environment
[localization_benchmark_supervisor-14] got robot radius
[gzserver-2] [Wrn] [Publisher.cc:141] Queue limit reached for topic /gazebo/default/pose/local/info, deleting message. This warning is printed only once.
[localization_benchmark_supervisor-14] called pause_physics_service
[localization_benchmark_supervisor-14] called set_entity_state_service
[localization_benchmark_supervisor-14] called unpause_physics_service
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Starting managed nodes bringup...
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Configuring map_server
[map_server-12] [INFO] [map_server]: Configuring
[map_server-12] [INFO] [map_server]: OccGridLoader: Creating
[map_server-12] [INFO] [map_server]: OccGridLoader: Configuring
[map_server-12] [INFO] [map_server]: Loading yaml file: /home/enrico/ds/performance_modelling/test_datasets/dataset/7A-2/data/map.yaml
[map_server-12] [INFO] [map_server]: Loading image_file: /home/enrico/ds/performance_modelling/test_datasets/dataset/7A-2/data/map.pgm
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Configuring amcl
[amcl-13] [INFO] [amcl]: Configuring
[amcl-13] [INFO] [amcl]: initTransforms
[amcl-13] [INFO] [amcl]: initPubSub
[amcl-13] [INFO] [amcl]: Subscribed to map topic.
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Configuring controller_server
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Configuring planner_server
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Configuring recoveries_server
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Configuring bt_navigator
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Configuring waypoint_follower
[waypoint_follower-11] [WARN] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Activating map_server
[map_server-12] [INFO] [map_server]: Activating
[map_server-12] [INFO] [map_server]: OccGridLoader: Activating
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Activating amcl
[amcl-13] [INFO] [amcl]: Activating
[amcl-13] [INFO] [amcl]: initialPoseReceived
[amcl-13] [INFO] [amcl]: Setting pose (12.626000): 7.975 41.100 -1.650
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Activating controller_server
[amcl-13] [INFO] [amcl]: Received a 487 X 1784 map @ 0.050 m/pix
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Activating planner_server
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Activating recoveries_server
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Activating bt_navigator
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Activating waypoint_follower
[localization_benchmark_supervisor-14] t: 12.684, event: run_start
[localization_benchmark_supervisor-14] goal 1 / 161
[localization_benchmark_supervisor-14] t: 12.684, event: target_pose_set
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Managed nodes are active
[localization_benchmark_supervisor-14] t: 12.684, event: target_pose_accepted
[amcl-13] [INFO] [amcl]: createLaserObject
[controller_server-7] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[controller_server-7] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[controller_server-7] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[localization_benchmark_supervisor-14] t: 16.183, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 2 / 161
[localization_benchmark_supervisor-14] t: 16.183, event: target_pose_set
[localization_benchmark_supervisor-14] t: 16.183, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 21.675, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 3 / 161
[localization_benchmark_supervisor-14] t: 21.675, event: target_pose_set
[localization_benchmark_supervisor-14] t: 21.675, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 34.253, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 4 / 161
[localization_benchmark_supervisor-14] t: 34.253, event: target_pose_set
[localization_benchmark_supervisor-14] t: 34.253, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 48.951, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 5 / 161
[localization_benchmark_supervisor-14] t: 48.951, event: target_pose_set
[localization_benchmark_supervisor-14] t: 48.951, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 63.281, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 6 / 161
[localization_benchmark_supervisor-14] t: 63.281, event: target_pose_set
[localization_benchmark_supervisor-14] t: 63.281, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 98.049, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 7 / 161
[localization_benchmark_supervisor-14] t: 98.049, event: target_pose_set
[localization_benchmark_supervisor-14] t: 98.049, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 111.337, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 8 / 161
[localization_benchmark_supervisor-14] t: 111.337, event: target_pose_set
[localization_benchmark_supervisor-14] t: 111.337, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 129.298, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 9 / 161
[localization_benchmark_supervisor-14] t: 129.298, event: target_pose_set
[localization_benchmark_supervisor-14] t: 129.298, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 133.198, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 10 / 161
[localization_benchmark_supervisor-14] t: 133.198, event: target_pose_set
[localization_benchmark_supervisor-14] t: 133.198, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 139.957, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 11 / 161
[localization_benchmark_supervisor-14] t: 139.957, event: target_pose_set
[localization_benchmark_supervisor-14] t: 139.957, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 157.919, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 12 / 161
[localization_benchmark_supervisor-14] t: 157.919, event: target_pose_set
[localization_benchmark_supervisor-14] t: 157.919, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 165.809, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 13 / 161
[localization_benchmark_supervisor-14] t: 165.809, event: target_pose_set
[localization_benchmark_supervisor-14] t: 165.809, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 169.893, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 14 / 161
[localization_benchmark_supervisor-14] t: 169.893, event: target_pose_set
[localization_benchmark_supervisor-14] t: 169.893, event: target_pose_accepted
[controller_server-7] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[localization_benchmark_supervisor-14] t: 182.314, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 15 / 161
[localization_benchmark_supervisor-14] t: 182.314, event: target_pose_set
[localization_benchmark_supervisor-14] t: 182.314, event: target_pose_accepted
[controller_server-7] [ERROR] [controller_server]: Failed to make progress
[controller_server-7] [WARN] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
[localization_benchmark_supervisor-14] t: 208.828, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 16 / 161
[localization_benchmark_supervisor-14] t: 208.828, event: target_pose_set
[localization_benchmark_supervisor-14] t: 208.828, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 225.274, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 17 / 161
[localization_benchmark_supervisor-14] t: 225.274, event: target_pose_set
[localization_benchmark_supervisor-14] t: 225.274, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 241.667, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 18 / 161
[localization_benchmark_supervisor-14] t: 241.667, event: target_pose_set
[localization_benchmark_supervisor-14] t: 241.667, event: target_pose_accepted
[controller_server-7] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[localization_benchmark_supervisor-14] t: 283.731, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 19 / 161
[localization_benchmark_supervisor-14] t: 283.731, event: target_pose_set
[localization_benchmark_supervisor-14] t: 283.731, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 326.613, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 20 / 161
[localization_benchmark_supervisor-14] t: 326.613, event: target_pose_set
[localization_benchmark_supervisor-14] t: 326.613, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 333.104, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 21 / 161
[localization_benchmark_supervisor-14] t: 333.104, event: target_pose_set
[localization_benchmark_supervisor-14] t: 333.104, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 348.087, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 22 / 161
[localization_benchmark_supervisor-14] t: 348.087, event: target_pose_set
[localization_benchmark_supervisor-14] t: 348.087, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 357.338, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 23 / 161
[localization_benchmark_supervisor-14] t: 357.338, event: target_pose_set
[localization_benchmark_supervisor-14] t: 357.338, event: target_pose_accepted
[controller_server-7] [ERROR] [controller_server]: Failed to make progress
[controller_server-7] [WARN] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
[controller_server-7] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[controller_server-7] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[controller_server-7] [ERROR] [controller_server]: Failed to make progress
[controller_server-7] [WARN] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
[localization_benchmark_supervisor-14] t: 395.523, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 24 / 161
[localization_benchmark_supervisor-14] t: 395.523, event: target_pose_set
[localization_benchmark_supervisor-14] t: 395.523, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 404.451, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 25 / 161
[localization_benchmark_supervisor-14] t: 404.451, event: target_pose_set
[localization_benchmark_supervisor-14] t: 404.451, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 416.82, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 26 / 161
[localization_benchmark_supervisor-14] t: 416.82, event: target_pose_set
[localization_benchmark_supervisor-14] t: 416.82, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 447.261, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 27 / 161
[localization_benchmark_supervisor-14] t: 447.261, event: target_pose_set
[localization_benchmark_supervisor-14] t: 447.261, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 499.618, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 28 / 161
[localization_benchmark_supervisor-14] t: 499.618, event: target_pose_set
[localization_benchmark_supervisor-14] t: 499.618, event: target_pose_accepted
[controller_server-7] [ERROR] [controller_server]: Failed to make progress
[controller_server-7] [WARN] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
[localization_benchmark_supervisor-14] t: 520.119, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 29 / 161
[localization_benchmark_supervisor-14] t: 520.119, event: target_pose_set
[localization_benchmark_supervisor-14] t: 520.273, event: target_pose_accepted
[localization_benchmark_supervisor-14] t: 533.311, event: target_pose_reached
[localization_benchmark_supervisor-14] goal 30 / 161
[localization_benchmark_supervisor-14] t: 533.311, event: target_pose_set
[localization_benchmark_supervisor-14] t: 533.311, event: target_pose_accepted
[ERROR] [bt_navigator-10]: process has died [pid 26318, exit code -11, cmd '/opt/ros/eloquent/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmpettddn0r        '].
[INFO] [lifecycle_manager-15]: sending signal 'SIGINT' to process[lifecycle_manager-15]
[INFO] [localization_benchmark_supervisor-14]: sending signal 'SIGINT' to process[localization_benchmark_supervisor-14]
[INFO] [amcl-13]: sending signal 'SIGINT' to process[amcl-13]
[INFO] [map_server-12]: sending signal 'SIGINT' to process[map_server-12]
[INFO] [waypoint_follower-11]: sending signal 'SIGINT' to process[waypoint_follower-11]
[INFO] [map_server-12]: process has finished cleanly [pid 26337]
[INFO] [recoveries_server-9]: sending signal 'SIGINT' to process[recoveries_server-9]
[INFO] [planner_server-8]: sending signal 'SIGINT' to process[planner_server-8]
[INFO] [controller_server-7]: sending signal 'SIGINT' to process[controller_server-7]
[INFO] [recoveries_server-9]: process has finished cleanly [pid 26316]
[INFO] [robot_state_publisher-6]: sending signal 'SIGINT' to process[robot_state_publisher-6]
[INFO] [static_transform_publisher-5]: sending signal 'SIGINT' to process[static_transform_publisher-5]
[INFO] [controller_server-7]: process has finished cleanly [pid 26303]
[INFO] [planner_server-8]: process has finished cleanly [pid 26308]
[INFO] [lifecycle_manager-15]: process has finished cleanly [pid 26382]
[INFO] [robot_state_publisher-4]: sending signal 'SIGINT' to process[robot_state_publisher-4]
[INFO] [amcl-13]: process has finished cleanly [pid 26361]
[INFO] [gzclient-3]: sending signal 'SIGINT' to process[gzclient-3]
[INFO] [gzserver-2]: sending signal 'SIGINT' to process[gzserver-2]
[INFO] [rviz2-1]: sending signal 'SIGINT' to process[rviz2-1]
[INFO] [static_transform_publisher-5]: process has finished cleanly [pid 26293]
[INFO] [waypoint_follower-11]: process has finished cleanly [pid 26330]
[lifecycle_manager-15] [INFO] [rclcpp]: signal_handler(signal_value=2)
[localization_benchmark_supervisor-14] Traceback (most recent call last):
[localization_benchmark_supervisor-14]   File "/home/enrico/w/ros2_ws/build/localization_performance_modelling/localization_performance_modelling/localization_benchmark_supervisor.py", line 58, in main
[localization_benchmark_supervisor-14]     rclpy.spin(node)
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/__init__.py", line 190, in spin
[localization_benchmark_supervisor-14]     executor.spin_once()
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 684, in spin_once
[localization_benchmark_supervisor-14]     raise handler.exception()
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/task.py", line 239, in __call__
[INFO] [robot_state_publisher-6]: process has finished cleanly [pid 26294]
[lifecycle_manager-15] [INFO] [lifecycle_manager]: Destroying
[localization_benchmark_supervisor-14]     self._handler.send(None)
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 404, in handler
[localization_benchmark_supervisor-14]     await call_coroutine(entity, arg)
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 330, in _execute_subscription
[localization_benchmark_supervisor-14]     await await_or_execute(sub.callback, msg)
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/rclpy/executors.py", line 118, in await_or_execute
[localization_benchmark_supervisor-14]     return callback(*args)
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/tf2_ros/transform_listener.py", line 105, in callback
[localization_benchmark_supervisor-14]     self.buffer.set_transform(transform, who)
[localization_benchmark_supervisor-14]   File "/opt/ros/eloquent/lib/python3.6/site-packages/tf2_ros/buffer.py", line 78, in set_transform
[localization_benchmark_supervisor-14]     super().set_transform(*args, **kwargs)
[localization_benchmark_supervisor-14] SystemError: ../Objects/longobject.c:568: bad argument to internal function
[localization_benchmark_supervisor-14] 
[amcl-13] [INFO] [rclcpp]: signal_handler(signal_value=2)
[amcl-13] [INFO] [amcl]: Destroying
[map_server-12] [INFO] [rclcpp]: signal_handler(signal_value=2)
[INFO] [robot_state_publisher-4]: process has finished cleanly [pid 26292]
[INFO] [localization_benchmark_supervisor-14]: process has finished cleanly [pid 26371]
[INFO] [gzclient-3]: process has finished cleanly [pid 26291]
[ERROR] [gzserver-2]: process[gzserver-2] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
[ERROR] [rviz2-1]: process[rviz2-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
[INFO] [gzserver-2]: sending signal 'SIGTERM' to process[gzserver-2]
[INFO] [rviz2-1]: sending signal 'SIGTERM' to process[rviz2-1]
[ERROR] [rviz2-1]: process has died [pid 26289, exit code -15, cmd '/opt/ros/eloquent/lib/rviz2/rviz2 -d /home/enrico/w/ros2_ws/src/localization_performance_modelling/config/component_configurations/rviz/nav2_default_view.rviz --ros-args -r __node:=rviz2          '].
[ERROR] [gzserver-2]: process has died [pid 26290, exit code -15, cmd 'gzserver --verbose -s libgazebo_ros_init.so /home/enrico/ds/performance_modelling/output/test_localization/run_103/components_configuration/gazebo/gazebo_environment.model'].
[gzserver-2] 
t: 1591822930.9277308, run: 103, event: supervisor_shutdown
execute_run: components shutdown completed
t: 1591822930.9283128, run: 103, event: start_compute_metrics
trajectory_length
relative_localization_correction_error
relative_localization_error
absolute_localization_correction_error
absolute_localization_error
cpu_and_memory_usage
visualisation
t: 1591822979.7644854, run: 103, event: run_end
run 103 completed
benchmark: run 103 completed
``` 

# AMCL Error after setting initial pose a few times
```
[amcl-11] [INFO] [amcl]: initialPoseReceived
[amcl-11] [INFO] [amcl]: Setting pose (300.411000): -0.525 -2.752 0.996
[amcl-11] amcl: /tmp/binarydeb/ros-eloquent-nav2-amcl-0.3.3/src/pf/pf.c:379: pf_update_resample: Assertion `i < set_a->sample_count' failed.
[ERROR] [amcl-11]: process has died [pid 27580, exit code -6, cmd '/opt/ros/eloquent/lib/nav2_amcl/amcl --ros-args -r __node:=amcl --params-file /tmp/tmpk6kmuu71        '].
```

# BT Navigator error after goal pose has been accepted
```
benchmark: starting run 5
	environment_folder: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab
	parameters_combination_dict:
		beta: [0.0175, 0.0175, 0.005, 0.005]
		laser_scan_max_range: 5.0
		laser_scan_fov_deg: 180
execute_run: launching components
execute_run: waiting for supervisor to finish
[INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [localization_benchmark_supervisor-77]: process started with pid [6774]
[INFO] [gzserver-66]: process started with pid [6690]
[INFO] [robot_state_publisher-67]: process started with pid [6691]
[INFO] [static_transform_publisher-68]: process started with pid [6692]
[INFO] [robot_state_publisher-69]: process started with pid [6693]
[INFO] [controller_server-70]: process started with pid [6695]
[INFO] [planner_server-71]: process started with pid [6707]
[INFO] [recoveries_server-72]: process started with pid [6714]
[INFO] [bt_navigator-73]: process started with pid [6715]
[INFO] [waypoint_follower-74]: process started with pid [6724]
[INFO] [map_server-75]: process started with pid [6742]
[INFO] [amcl-76]: process started with pid [6760]
[INFO] [lifecycle_manager-78]: process started with pid [6793]
[robot_state_publisher-67] Initialize urdf model from file: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab/gazebo/robot_gt.urdf
[robot_state_publisher-67] Parsing robot urdf xml string.
[robot_state_publisher-67] Link base_link had 7 children
[robot_state_publisher-67] Link camera_link_gt had 2 children
[robot_state_publisher-67] Link camera_depth_frame_gt had 1 children
[robot_state_publisher-67] Link camera_depth_optical_frame_gt had 0 children
[robot_state_publisher-67] Link camera_rgb_frame_gt had 1 children
[robot_state_publisher-67] Link camera_rgb_optical_frame_gt had 0 children
[robot_state_publisher-67] Link caster_back_left_link_gt had 0 children
[robot_state_publisher-67] Link caster_back_right_link_gt had 0 children
[robot_state_publisher-67] Link imu_link_gt had 0 children
[robot_state_publisher-67] Link base_scan_gt had 0 children
[robot_state_publisher-67] Link wheel_left_link_gt had 0 children
[robot_state_publisher-67] Link wheel_right_link_gt had 0 children
[robot_state_publisher-67] got segment base_footprint_gt
[robot_state_publisher-67] got segment base_link
[robot_state_publisher-67] got segment base_scan_gt
[robot_state_publisher-67] got segment camera_depth_frame_gt
[robot_state_publisher-67] got segment camera_depth_optical_frame_gt
[robot_state_publisher-67] got segment camera_link_gt
[robot_state_publisher-67] got segment camera_rgb_frame_gt
[robot_state_publisher-67] got segment camera_rgb_optical_frame_gt
[robot_state_publisher-67] got segment caster_back_left_link_gt
[robot_state_publisher-67] got segment caster_back_right_link_gt
[robot_state_publisher-67] got segment imu_link_gt
[robot_state_publisher-67] got segment wheel_left_link_gt
[robot_state_publisher-67] got segment wheel_right_link_gt
[robot_state_publisher-67] Adding fixed segment from base_footprint_gt to base_link
[robot_state_publisher-67] Adding fixed segment from base_link to camera_link_gt
[robot_state_publisher-67] Adding fixed segment from camera_link_gt to camera_depth_frame_gt
[robot_state_publisher-67] Adding fixed segment from camera_depth_frame_gt to camera_depth_optical_frame_gt
[robot_state_publisher-67] Adding fixed segment from camera_link_gt to camera_rgb_frame_gt
[robot_state_publisher-67] Adding fixed segment from camera_rgb_frame_gt to camera_rgb_optical_frame_gt
[robot_state_publisher-67] Adding fixed segment from base_link to caster_back_left_link_gt
[robot_state_publisher-67] Adding fixed segment from base_link to caster_back_right_link_gt
[robot_state_publisher-67] Adding fixed segment from base_link to imu_link_gt
[robot_state_publisher-67] Adding fixed segment from base_link to base_scan_gt
[robot_state_publisher-67] Adding moving segment from base_link to wheel_left_link_gt
[robot_state_publisher-67] Adding moving segment from base_link to wheel_right_link_gt
[robot_state_publisher-69] Initialize urdf model from file: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab/gazebo/robot_realistic.urdf
[robot_state_publisher-69] Parsing robot urdf xml string.
[robot_state_publisher-69] Link base_link_realistic had 7 children
[robot_state_publisher-69] Link camera_link_realistic had 2 children
[robot_state_publisher-69] Link camera_depth_frame_realistic had 1 children
[robot_state_publisher-69] Link camera_depth_optical_frame_realistic had 0 children
[robot_state_publisher-69] Link camera_rgb_frame_realistic had 1 children
[robot_state_publisher-69] Link camera_rgb_optical_frame_realistic had 0 children
[robot_state_publisher-69] Link caster_back_left_link_realistic had 0 children
[robot_state_publisher-69] Link caster_back_right_link_realistic had 0 children
[robot_state_publisher-69] Link imu_link_realistic had 0 children
[robot_state_publisher-69] Link base_scan_realistic had 0 children
[robot_state_publisher-69] Link wheel_left_link_realistic had 0 children
[robot_state_publisher-69] Link wheel_right_link_realistic had 0 children
[robot_state_publisher-69] got segment base_footprint_realistic
[robot_state_publisher-69] got segment base_link_realistic
[robot_state_publisher-69] got segment base_scan_realistic
[robot_state_publisher-69] got segment camera_depth_frame_realistic
[robot_state_publisher-69] got segment camera_depth_optical_frame_realistic
[robot_state_publisher-69] got segment camera_link_realistic
[robot_state_publisher-69] got segment camera_rgb_frame_realistic
[robot_state_publisher-69] got segment camera_rgb_optical_frame_realistic
[robot_state_publisher-69] got segment caster_back_left_link_realistic
[robot_state_publisher-69] got segment caster_back_right_link_realistic
[robot_state_publisher-69] got segment imu_link_realistic
[robot_state_publisher-69] got segment wheel_left_link_realistic
[robot_state_publisher-69] got segment wheel_right_link_realistic
[robot_state_publisher-69] Adding fixed segment from base_footprint_realistic to base_link_realistic
[robot_state_publisher-69] Adding fixed segment from base_link_realistic to camera_link_realistic
[robot_state_publisher-69] Adding fixed segment from camera_link_realistic to camera_depth_frame_realistic
[robot_state_publisher-69] Adding fixed segment from camera_depth_frame_realistic to camera_depth_optical_frame_realistic
[robot_state_publisher-69] Adding fixed segment from camera_link_realistic to camera_rgb_frame_realistic
[robot_state_publisher-69] Adding fixed segment from camera_rgb_frame_realistic to camera_rgb_optical_frame_realistic
[robot_state_publisher-69] Adding fixed segment from base_link_realistic to caster_back_left_link_realistic
[robot_state_publisher-69] Adding fixed segment from base_link_realistic to caster_back_right_link_realistic
[robot_state_publisher-69] Adding fixed segment from base_link_realistic to imu_link_realistic
[robot_state_publisher-69] Adding fixed segment from base_link_realistic to base_scan_realistic
[robot_state_publisher-69] Adding moving segment from base_link_realistic to wheel_left_link_realistic
[robot_state_publisher-69] Adding moving segment from base_link_realistic to wheel_right_link_realistic
[map_server-75] [INFO] [map_server]: Creating
[amcl-76] [INFO] [amcl]: Creating
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Creating
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Creating and initializing lifecycle service clients
[gzserver-66] Gazebo multi-robot simulator, version 9.0.0
[gzserver-66] Copyright (C) 2012 Open Source Robotics Foundation.
[gzserver-66] Released under the Apache 2 License.
[gzserver-66] http://gazebosim.org
[gzserver-66] 
[localization_benchmark_supervisor-77] preparing to start run
[gzserver-66] [Msg] Waiting for master.
[gzserver-66] [Msg] Connected to gazebo master @ http://127.0.0.1:11345
[gzserver-66] [Msg] Publicized address: 192.168.1.101
[gzserver-66] [INFO] [camera_driver]: Publishing camera info to [/camera/camera_info]
[gzserver-66] [INFO] [gazebo_ros_state]: Publishing states of gazebo models at [/model_states]
[gzserver-66] [INFO] [gazebo_ros_state]: Publishing states of gazebo links at [/link_states]
[gzserver-66] [INFO] [turtlebot3_diff_drive]: Wheel pair 1 separation set to [0.287000m]
[gzserver-66] [INFO] [turtlebot3_diff_drive]: Wheel pair 1 diameter set to [0.066000m]
[gzserver-66] [INFO] [turtlebot3_diff_drive]: Subscribed to [/cmd_vel]
[gzserver-66] [INFO] [turtlebot3_diff_drive]: Computing odometry with parametric error model
[gzserver-66] alpha1 [0.017500]
[gzserver-66] alpha2 [0.017500]
[gzserver-66] alpha3 [0.005000]
[gzserver-66] alpha4 [0.005000]
[gzserver-66] 
[gzserver-66] [INFO] [turtlebot3_diff_drive]: Advertise odometry on [/odom]
[gzserver-66] [INFO] [turtlebot3_diff_drive]: Publishing odom transforms between [odom_realistic] and [base_footprint_realistic]
[gzserver-66] [INFO] [turtlebot3_diff_drive]: Publishing ground truth odom transforms between [odom] and [base_footprint_gt]
[gzserver-66] [INFO] [turtlebot3_joint_state]: Going to publish joint [robot::wheel_left_joint]
[gzserver-66] [INFO] [turtlebot3_joint_state]: Going to publish joint [robot::wheel_right_joint]
[localization_benchmark_supervisor-77] got robot radius
[localization_benchmark_supervisor-77] called pause_physics_service
[localization_benchmark_supervisor-77] called set_entity_state_service gazebo_msgs.srv.SetEntityState_Response(success=True)
[localization_benchmark_supervisor-77] called unpause_physics_service
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Starting managed nodes bringup...
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Configuring map_server
[map_server-75] [INFO] [map_server]: Configuring
[map_server-75] [INFO] [map_server]: OccGridLoader: Creating
[map_server-75] [INFO] [map_server]: OccGridLoader: Configuring
[map_server-75] [INFO] [map_server]: Loading yaml file: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab/data/map.yaml
[map_server-75] [INFO] [map_server]: Loading image_file: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab/data/map.pgm
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Configuring amcl
[amcl-76] [INFO] [amcl]: Configuring
[amcl-76] [INFO] [amcl]: initTransforms
[amcl-76] [INFO] [amcl]: initPubSub
[amcl-76] [INFO] [amcl]: Subscribed to map topic.
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Configuring controller_server
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Configuring planner_server
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Configuring recoveries_server
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Configuring bt_navigator
[amcl-76] [INFO] [amcl_rclcpp_node]: Message Filter dropping message: frame 'base_scan_realistic' at time 5.600 for reason 'Unknown'
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Configuring waypoint_follower
[waypoint_follower-74] [WARN] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Activating map_server
[map_server-75] [INFO] [map_server]: Activating
[map_server-75] [INFO] [map_server]: OccGridLoader: Activating
[amcl-76] [INFO] [amcl]: Received a 211 X 215 map @ 0.050 m/pix
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Activating amcl
[amcl-76] [INFO] [amcl]: Activating
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Activating controller_server
[amcl-76] [INFO] [amcl]: initialPoseReceived
[amcl-76] [INFO] [amcl]: Setting pose (6.435000): -1.151 -3.656 -0.667
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Activating planner_server
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Activating recoveries_server
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Activating bt_navigator
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Activating waypoint_follower
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Managed nodes are active
[localization_benchmark_supervisor-77] t: 6.477, event: run_start
[localization_benchmark_supervisor-77] t: 6.477, event: target_pose_set
[localization_benchmark_supervisor-77] t: 6.477, event: target_pose_accepted
[amcl-76] [INFO] [amcl]: createLaserObject
[gzserver-66] [Wrn] [Publisher.cc:141] Queue limit reached for topic /gazebo/default/pose/local/info, deleting message. This warning is printed only once.
[controller_server-70] [ERROR] [controller_server]: Failed to make progress
[controller_server-70] [WARN] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
[localization_benchmark_supervisor-77] t: 28.314, event: target_pose_reached
[localization_benchmark_supervisor-77] t: 28.314, event: target_pose_set
[localization_benchmark_supervisor-77] t: 28.314, event: target_pose_accepted
[controller_server-70] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
[localization_benchmark_supervisor-77] t: 46.402, event: target_pose_reached
[localization_benchmark_supervisor-77] t: 46.402, event: target_pose_set
[localization_benchmark_supervisor-77] t: 46.402, event: target_pose_accepted
[ERROR] [bt_navigator-73]: process has died [pid 6715, exit code -11, cmd '/opt/ros/eloquent/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmphx3vj9av        '].
[localization_benchmark_supervisor-77] terminating supervisor due to timeout, terminating run
[localization_benchmark_supervisor-77] t: 10800.035, event: run_timeout
[localization_benchmark_supervisor-77] t: 10800.035, event: supervisor_finished
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[lifecycle_manager-78] [INFO] [rclcpp]: signal_handler(signal_value=2)
[amcl-76] [INFO] [rclcpp]: signal_handler(signal_value=2)
[map_server-75] [INFO] [rclcpp]: signal_handler(signal_value=2)
[localization_benchmark_supervisor-77] asked to shutdown, terminating run
[localization_benchmark_supervisor-77] t: 10800.035, event: ros_shutdown
[localization_benchmark_supervisor-77] t: 10800.035, event: supervisor_finished
[lifecycle_manager-78] [INFO] [lifecycle_manager]: Destroying
[amcl-76] [INFO] [amcl]: Destroying
[INFO] [recoveries_server-72]: process has finished cleanly [pid 6714]
[INFO] [map_server-75]: process has finished cleanly [pid 6742]
[INFO] [static_transform_publisher-68]: process has finished cleanly [pid 6692]
[INFO] [robot_state_publisher-69]: process has finished cleanly [pid 6693]
[INFO] [robot_state_publisher-67]: process has finished cleanly [pid 6691]
[INFO] [controller_server-70]: process has finished cleanly [pid 6695]
[INFO] [planner_server-71]: process has finished cleanly [pid 6707]
[INFO] [waypoint_follower-74]: process has finished cleanly [pid 6724]
[INFO] [lifecycle_manager-78]: process has finished cleanly [pid 6793]
[INFO] [amcl-76]: process has finished cleanly [pid 6760]
[INFO] [gzserver-66]: process has finished cleanly [pid 6690]
[gzserver-66] 
[gzserver-66] 
[INFO] [localization_benchmark_supervisor-77]: process has finished cleanly [pid 6774]
```
Ros launch.log files:
```
1590973618.6826069 [INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
1590973618.6828170 [INFO] [launch]: Default logging verbosity is set to INFO
1590973618.6829443 [INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
1590973618.6830182 [INFO] [launch]: Default logging verbosity is set to INFO
1590973618.6831198 [INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
1590973618.6832380 [INFO] [launch]: Default logging verbosity is set to INFO
1590973618.6833344 [INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-01-02-41-18-701405-enrico-zenbook-2874
1590973618.6834004 [INFO] [launch]: Default logging verbosity is set to INFO
1590973619.0405529 [INFO] [localization_benchmark_supervisor-77]: process started with pid [6774]
1590973619.0421283 [INFO] [gzserver-66]: process started with pid [6690]
1590973619.0424430 [INFO] [robot_state_publisher-67]: process started with pid [6691]
1590973619.0426080 [INFO] [static_transform_publisher-68]: process started with pid [6692]
1590973619.0427971 [INFO] [robot_state_publisher-69]: process started with pid [6693]
1590973619.0429456 [INFO] [controller_server-70]: process started with pid [6695]
1590973619.0431094 [INFO] [planner_server-71]: process started with pid [6707]
1590973619.0432708 [INFO] [recoveries_server-72]: process started with pid [6714]
1590973619.0434041 [INFO] [bt_navigator-73]: process started with pid [6715]
1590973619.0441561 [INFO] [waypoint_follower-74]: process started with pid [6724]
1590973619.0443587 [INFO] [map_server-75]: process started with pid [6742]
1590973619.0445259 [INFO] [amcl-76]: process started with pid [6760]
1590973619.0447910 [INFO] [lifecycle_manager-78]: process started with pid [6793]
1590973619.0455530 [controller_server-70] [INFO] [controller_server]: Creating controller server
1590973619.0456545 [controller_server-70] [INFO] [local_costmap.local_costmap]: Creating Costmap
1590973619.0464990 [static_transform_publisher-68] [INFO] [gt_odom_static_transform_publisher]: Spinning until killed publishing map to odom
1590973619.0468552 [planner_server-71] [INFO] [planner_server]: Creating
1590973619.0469518 [planner_server-71] [INFO] [global_costmap.global_costmap]: Creating Costmap
1590973619.0478511 [robot_state_publisher-67] Initialize urdf model from file: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab/gazebo/robot_gt.urdf
1590973619.0503860 [robot_state_publisher-67] Parsing robot urdf xml string.
1590973619.0505946 [robot_state_publisher-67] Link base_link had 7 children
1590973619.0507920 [robot_state_publisher-67] Link camera_link_gt had 2 children
1590973619.0518363 [robot_state_publisher-67] Link camera_depth_frame_gt had 1 children
1590973619.0579984 [robot_state_publisher-67] Link camera_depth_optical_frame_gt had 0 children
1590973619.0591590 [robot_state_publisher-67] Link camera_rgb_frame_gt had 1 children
1590973619.0593834 [robot_state_publisher-67] Link camera_rgb_optical_frame_gt had 0 children
1590973619.0595911 [robot_state_publisher-67] Link caster_back_left_link_gt had 0 children
1590973619.0600100 [robot_state_publisher-67] Link caster_back_right_link_gt had 0 children
1590973619.0607693 [robot_state_publisher-67] Link imu_link_gt had 0 children
1590973619.0615022 [robot_state_publisher-67] Link base_scan_gt had 0 children
1590973619.0638065 [robot_state_publisher-67] Link wheel_left_link_gt had 0 children
1590973619.0647614 [robot_state_publisher-67] Link wheel_right_link_gt had 0 children
1590973619.0717847 [robot_state_publisher-67] got segment base_footprint_gt
1590973619.0722847 [robot_state_publisher-67] got segment base_link
1590973619.0731308 [robot_state_publisher-67] got segment base_scan_gt
1590973619.0732758 [robot_state_publisher-67] got segment camera_depth_frame_gt
1590973619.0733886 [robot_state_publisher-67] got segment camera_depth_optical_frame_gt
1590973619.0734842 [robot_state_publisher-67] got segment camera_link_gt
1590973619.0738649 [robot_state_publisher-67] got segment camera_rgb_frame_gt
1590973619.0739763 [robot_state_publisher-67] got segment camera_rgb_optical_frame_gt
1590973619.0786712 [robot_state_publisher-67] got segment caster_back_left_link_gt
1590973619.0792592 [robot_state_publisher-67] got segment caster_back_right_link_gt
1590973619.0830722 [robot_state_publisher-67] got segment imu_link_gt
1590973619.0833457 [robot_state_publisher-67] got segment wheel_left_link_gt
1590973619.0834582 [robot_state_publisher-67] got segment wheel_right_link_gt
1590973619.0836585 [robot_state_publisher-67] Adding fixed segment from base_footprint_gt to base_link
1590973619.0838697 [robot_state_publisher-67] Adding fixed segment from base_link to camera_link_gt
1590973619.0839734 [robot_state_publisher-67] Adding fixed segment from camera_link_gt to camera_depth_frame_gt
1590973619.0845764 [robot_state_publisher-67] Adding fixed segment from camera_depth_frame_gt to camera_depth_optical_frame_gt
1590973619.0847125 [robot_state_publisher-67] Adding fixed segment from camera_link_gt to camera_rgb_frame_gt
1590973619.0848341 [robot_state_publisher-67] Adding fixed segment from camera_rgb_frame_gt to camera_rgb_optical_frame_gt
1590973619.0851965 [robot_state_publisher-67] Adding fixed segment from base_link to caster_back_left_link_gt
1590973619.0854373 [robot_state_publisher-67] Adding fixed segment from base_link to caster_back_right_link_gt
1590973619.0856838 [robot_state_publisher-67] Adding fixed segment from base_link to imu_link_gt
1590973619.0858660 [robot_state_publisher-67] Adding fixed segment from base_link to base_scan_gt
1590973619.0860574 [robot_state_publisher-67] Adding moving segment from base_link to wheel_left_link_gt
1590973619.0862069 [robot_state_publisher-67] Adding moving segment from base_link to wheel_right_link_gt
1590973619.0874677 [robot_state_publisher-69] Initialize urdf model from file: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab/gazebo/robot_realistic.urdf
1590973619.0919061 [robot_state_publisher-69] Parsing robot urdf xml string.
1590973619.0999882 [robot_state_publisher-69] Link base_link_realistic had 7 children
1590973619.1061676 [robot_state_publisher-69] Link camera_link_realistic had 2 children
1590973619.1099541 [robot_state_publisher-69] Link camera_depth_frame_realistic had 1 children
1590973619.1110544 [robot_state_publisher-69] Link camera_depth_optical_frame_realistic had 0 children
1590973619.1112642 [robot_state_publisher-69] Link camera_rgb_frame_realistic had 1 children
1590973619.1114259 [robot_state_publisher-69] Link camera_rgb_optical_frame_realistic had 0 children
1590973619.1115761 [robot_state_publisher-69] Link caster_back_left_link_realistic had 0 children
1590973619.1117227 [robot_state_publisher-69] Link caster_back_right_link_realistic had 0 children
1590973619.1118698 [robot_state_publisher-69] Link imu_link_realistic had 0 children
1590973619.1120522 [robot_state_publisher-69] Link base_scan_realistic had 0 children
1590973619.1122038 [robot_state_publisher-69] Link wheel_left_link_realistic had 0 children
1590973619.1123428 [robot_state_publisher-69] Link wheel_right_link_realistic had 0 children
1590973619.1124830 [robot_state_publisher-69] got segment base_footprint_realistic
1590973619.1126363 [robot_state_publisher-69] got segment base_link_realistic
1590973619.1127799 [robot_state_publisher-69] got segment base_scan_realistic
1590973619.1129246 [robot_state_publisher-69] got segment camera_depth_frame_realistic
1590973619.1130979 [robot_state_publisher-69] got segment camera_depth_optical_frame_realistic
1590973619.1153581 [robot_state_publisher-69] got segment camera_link_realistic
1590973619.1163738 [robot_state_publisher-69] got segment camera_rgb_frame_realistic
1590973619.1169329 [robot_state_publisher-69] got segment camera_rgb_optical_frame_realistic
1590973619.1174822 [robot_state_publisher-69] got segment caster_back_left_link_realistic
1590973619.1225657 [robot_state_publisher-69] got segment caster_back_right_link_realistic
1590973619.1258621 [robot_state_publisher-69] got segment imu_link_realistic
1590973619.1275659 [robot_state_publisher-69] got segment wheel_left_link_realistic
1590973619.1284907 [robot_state_publisher-69] got segment wheel_right_link_realistic
1590973619.1310701 [robot_state_publisher-69] Adding fixed segment from base_footprint_realistic to base_link_realistic
1590973619.1360655 [robot_state_publisher-69] Adding fixed segment from base_link_realistic to camera_link_realistic
1590973619.1449456 [robot_state_publisher-69] Adding fixed segment from camera_link_realistic to camera_depth_frame_realistic
1590973619.1492522 [robot_state_publisher-69] Adding fixed segment from camera_depth_frame_realistic to camera_depth_optical_frame_realistic
1590973619.1496031 [robot_state_publisher-69] Adding fixed segment from camera_link_realistic to camera_rgb_frame_realistic
1590973619.1498184 [robot_state_publisher-69] Adding fixed segment from camera_rgb_frame_realistic to camera_rgb_optical_frame_realistic
1590973619.1499772 [robot_state_publisher-69] Adding fixed segment from base_link_realistic to caster_back_left_link_realistic
1590973619.1501386 [robot_state_publisher-69] Adding fixed segment from base_link_realistic to caster_back_right_link_realistic
1590973619.1503623 [robot_state_publisher-69] Adding fixed segment from base_link_realistic to imu_link_realistic
1590973619.1505284 [robot_state_publisher-69] Adding fixed segment from base_link_realistic to base_scan_realistic
1590973619.1508052 [robot_state_publisher-69] Adding moving segment from base_link_realistic to wheel_left_link_realistic
1590973619.1509917 [robot_state_publisher-69] Adding moving segment from base_link_realistic to wheel_right_link_realistic
1590973619.1513846 [waypoint_follower-74] [INFO] [waypoint_follower]: Creating
1590973619.1519930 [bt_navigator-73] [INFO] [bt_navigator]: Creating
1590973628.8109500 [controller_server-70] [INFO] [controller_server]: Configuring controller interface
1590973628.8110831 [controller_server-70] [INFO] [controller_server]: Controller frequency set to 20.0000Hz
1590973628.8114038 [controller_server-70] [INFO] [local_costmap.local_costmap]: Configuring
1590973628.8506703 [controller_server-70] [INFO] [local_costmap.local_costmap]: Using plugin "obstacle_layer"
1590973628.8589528 [controller_server-70] [INFO] [local_costmap.local_costmap]: Subscribed to Topics: scan
1590973628.8732829 [controller_server-70] [INFO] [local_costmap.local_costmap]: Initialized plugin "obstacle_layer"
1590973628.8846278 [controller_server-70] [INFO] [local_costmap.local_costmap]: Using plugin "inflation_layer"
1590973628.8873796 [controller_server-70] [INFO] [local_costmap.local_costmap]: Initializing inflation layer!
1590973628.8901834 [controller_server-70] [INFO] [local_costmap.local_costmap]: Initialized plugin "inflation_layer"
1590973628.9803970 [controller_server-70] [INFO] [controller_server]: Created controller : FollowPath of type dwb_core::DWBLocalPlanner
1590973628.9848375 [controller_server-70] [INFO] [controller_server]: Setting transform_tolerance to 0.200000
1590973629.0137627 [controller_server-70] [INFO] [controller_server]: Using critic "RotateToGoal" (dwb_critics::RotateToGoalCritic)
1590973629.0155952 [controller_server-70] [INFO] [controller_server]: Critic plugin initialized
1590973629.0167840 [controller_server-70] [INFO] [controller_server]: Using critic "Oscillation" (dwb_critics::OscillationCritic)
1590973629.0175753 [controller_server-70] [INFO] [controller_server]: Critic plugin initialized
1590973629.0200381 [controller_server-70] [INFO] [controller_server]: Using critic "BaseObstacle" (dwb_critics::BaseObstacleCritic)
1590973629.0209949 [controller_server-70] [INFO] [controller_server]: Critic plugin initialized
1590973629.0223281 [controller_server-70] [INFO] [controller_server]: Using critic "GoalAlign" (dwb_critics::GoalAlignCritic)
1590973629.0249310 [controller_server-70] [INFO] [controller_server]: Critic plugin initialized
1590973629.0265865 [controller_server-70] [INFO] [controller_server]: Using critic "PathAlign" (dwb_critics::PathAlignCritic)
1590973629.0283852 [controller_server-70] [INFO] [controller_server]: Critic plugin initialized
1590973629.0300431 [controller_server-70] [INFO] [controller_server]: Using critic "PathDist" (dwb_critics::PathDistCritic)
1590973629.0321379 [controller_server-70] [INFO] [controller_server]: Critic plugin initialized
1590973629.0336444 [controller_server-70] [INFO] [controller_server]: Using critic "GoalDist" (dwb_critics::GoalDistCritic)
1590973629.0342340 [controller_server-70] [INFO] [controller_server]: Critic plugin initialized
1590973629.0582099 [planner_server-71] [INFO] [planner_server]: Configuring
1590973629.0583220 [planner_server-71] [INFO] [global_costmap.global_costmap]: Configuring
1590973629.1058578 [planner_server-71] [INFO] [global_costmap.global_costmap]: Using plugin "static_layer"
1590973629.1126540 [planner_server-71] [INFO] [global_costmap.global_costmap]: Subscribing to the map topic (/map) with transient local durability
1590973629.1307893 [planner_server-71] [INFO] [global_costmap.global_costmap]: Initialized plugin "static_layer"
1590973629.1328921 [planner_server-71] [INFO] [global_costmap.global_costmap]: Using plugin "obstacle_layer"
1590973629.1369998 [planner_server-71] [INFO] [global_costmap.global_costmap]: Subscribed to Topics: scan
1590973629.1668215 [planner_server-71] [INFO] [global_costmap.global_costmap]: Initialized plugin "obstacle_layer"
1590973629.1692483 [planner_server-71] [INFO] [global_costmap.global_costmap]: Using plugin "inflation_layer"
1590973629.1712048 [planner_server-71] [INFO] [global_costmap.global_costmap]: Initializing inflation layer!
1590973629.1801872 [planner_server-71] [INFO] [global_costmap.global_costmap]: Initialized plugin "inflation_layer"
1590973629.2740195 [planner_server-71] [INFO] [planner_server]: Created global planner plugin GridBased of type nav2_navfn_planner/NavfnPlanner
1590973629.2747073 [planner_server-71] [INFO] [planner_server]: Configuring plugin GridBased of type NavfnPlanner
1590973629.3140135 [recoveries_server-72] [INFO] [recoveries_server]: Configuring
1590973629.3542776 [recoveries_server-72] [INFO] [recoveries_server]: Creating recovery plugin spin of type nav2_recoveries/Spin
1590973629.3565407 [recoveries_server-72] [INFO] [recoveries_server]: Configuring spin
1590973629.4191074 [recoveries_server-72] [INFO] [recoveries_server]: Creating recovery plugin back_up of type nav2_recoveries/BackUp
1590973629.4212177 [recoveries_server-72] [INFO] [recoveries_server]: Configuring back_up
1590973629.4482944 [recoveries_server-72] [INFO] [recoveries_server]: Creating recovery plugin wait of type nav2_recoveries/Wait
1590973629.4503968 [recoveries_server-72] [INFO] [recoveries_server]: Configuring wait
1590973629.4703462 [bt_navigator-73] [INFO] [bt_navigator]: Configuring
1590973629.7186611 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "compute_path_to_pose" action server
1590973629.7820718 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "ComputePathToPose" BtActionNode initialized
1590973629.7855756 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "global_costmap/clear_entirely_global_costmap" service
1590973630.7866650 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "ClearEntireCostmap" BtServiceNode initialized
1590973630.8011203 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "follow_path" action server
1590973630.8012424 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "FollowPath" BtActionNode initialized
1590973630.8072784 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "local_costmap/clear_entirely_local_costmap" service
1590973630.8074019 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "ClearEntireCostmap" BtServiceNode initialized
1590973630.8108108 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "local_costmap/clear_entirely_local_costmap" service
1590973630.8109341 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "ClearEntireCostmap" BtServiceNode initialized
1590973630.8154569 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "global_costmap/clear_entirely_global_costmap" service
1590973630.8156044 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "ClearEntireCostmap" BtServiceNode initialized
1590973630.8387468 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "spin" action server
1590973630.8390431 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "Spin" BtActionNode initialized
1590973630.8668058 [bt_navigator-73] [INFO] [bt_navigator_client_node]: Waiting for "wait" action server
1590973630.8671632 [bt_navigator-73] [INFO] [bt_navigator_client_node]: "Wait" BtActionNode initialized
1590973630.8690257 [waypoint_follower-74] [INFO] [waypoint_follower]: Configuring
1590973630.8909540 [waypoint_follower-74] [WARN] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
1590973630.9552979 [planner_server-71] [INFO] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 211 X 215 at 0.050000 m/pix
1590973630.9576826 [controller_server-70] [INFO] [controller_server]: Activating
1590973630.9578009 [controller_server-70] [INFO] [local_costmap.local_costmap]: Activating
1590973630.9578576 [controller_server-70] [INFO] [local_costmap.local_costmap]: Checking transform
1590973630.9579024 [controller_server-70] [INFO] [local_costmap.local_costmap]: start
1590973631.0072446 [planner_server-71] [INFO] [planner_server]: Activating
1590973631.0073526 [planner_server-71] [INFO] [global_costmap.global_costmap]: Activating
1590973631.0074084 [planner_server-71] [INFO] [global_costmap.global_costmap]: Checking transform
1590973631.0074549 [planner_server-71] [INFO] [global_costmap.global_costmap]: start
1590973631.0576336 [planner_server-71] [INFO] [planner_server]: Activating plugin GridBased of type NavfnPlanner
1590973631.0585635 [recoveries_server-72] [INFO] [recoveries_server]: Activating
1590973631.0593250 [bt_navigator-73] [INFO] [bt_navigator]: Activating
1590973631.0602798 [waypoint_follower-74] [INFO] [waypoint_follower]: Activating
1590973631.0679348 [bt_navigator-73] [INFO] [bt_navigator]: Begin navigating from current location to (-1.17, -4.07)
1590973631.0806997 [controller_server-70] [INFO] [controller_server]: Received a goal, begin computing control effort.
1590973632.1309452 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973633.1309471 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973634.1313629 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973635.1309903 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973636.1309836 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973637.1809969 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973638.1810277 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973639.2356324 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973640.2309611 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973641.2309821 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973642.2310839 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973643.2810347 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973644.2819490 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973644.3555074 [planner_server-71] [INFO] [global_costmap.global_costmap_rclcpp_node]: Message Filter dropping message: frame 'base_scan_gt' at time 5.400 for reason 'Unknown'
1590973645.2810230 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973645.6310496 [controller_server-70] [ERROR] [controller_server]: Failed to make progress
1590973645.6315463 [controller_server-70] [WARN] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
1590973645.6320295 [controller_server-70] [INFO] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap
1590973645.6323988 [controller_server-70] [INFO] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap
1590973645.6327460 [planner_server-71] [INFO] [global_costmap.global_costmap]: Received request to clear entirely the global_costmap
1590973645.6333163 [recoveries_server-72] [INFO] [recoveries_server]: Attempting spin
1590973645.6336615 [recoveries_server-72] [INFO] [recoveries_server]: Turning -1.57 for spin recovery.
1590973646.6337912 [recoveries_server-72] [INFO] [recoveries_server]: spin running...
1590973647.6338136 [recoveries_server-72] [INFO] [recoveries_server]: spin running...
1590973647.6341498 [recoveries_server-72] [INFO] [recoveries_server]: spin completed successfully
1590973647.6349180 [recoveries_server-72] [INFO] [recoveries_server]: Attempting wait
1590973648.6350768 [recoveries_server-72] [INFO] [recoveries_server]: wait running...
1590973649.6350908 [recoveries_server-72] [INFO] [recoveries_server]: wait running...
1590973650.6350133 [recoveries_server-72] [INFO] [recoveries_server]: wait running...
1590973651.6349320 [recoveries_server-72] [INFO] [recoveries_server]: wait running...
1590973652.6350579 [recoveries_server-72] [INFO] [recoveries_server]: wait running...
1590973652.6352057 [recoveries_server-72] [INFO] [recoveries_server]: wait completed successfully
1590973652.6378782 [controller_server-70] [INFO] [controller_server]: Received a goal, begin computing control effort.
1590973652.6881487 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973653.6883955 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973654.6919158 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973655.7382147 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973656.7389681 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973657.7381783 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973658.7382724 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973658.7946856 [controller_server-70] [INFO] [controller_server]: Reached the goal!
1590973658.7949827 [bt_navigator-73] [INFO] [bt_navigator]: Navigation succeeded
1590973658.7966743 [bt_navigator-73] [INFO] [bt_navigator]: Begin navigating from current location to (0.80, -1.30)
1590973658.7992897 [controller_server-70] [INFO] [controller_server]: Received a goal, begin computing control effort.
1590973658.8529871 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973659.8494809 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973660.7026212 [controller_server-70] [WARN] [controller_server]: Control loop missed its desired rate of 20.0000Hz
1590973660.8494933 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973661.9034553 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973662.8995557 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973663.8997040 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973664.8995638 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973665.8996515 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973666.8994908 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973667.9495802 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973668.9495153 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973669.9514492 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973670.9995754 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973671.9995790 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973672.9995620 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973673.9997590 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973674.9995360 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973675.9995189 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973676.9995565 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973678.0495186 [controller_server-70] [INFO] [controller_server]: Preempting the goal. Passing the new path to the planner.
1590973679.0254941 [controller_server-70] [INFO] [controller_server]: Reached the goal!
1590973679.0270393 [bt_navigator-73] [INFO] [bt_navigator]: Navigation succeeded
1590973679.0514903 [bt_navigator-73] [INFO] [bt_navigator]: Begin navigating from current location to (0.80, -0.75)
1590973679.0654733 [ERROR] [bt_navigator-73]: process has died [pid 6715, exit code -11, cmd '/opt/ros/eloquent/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmphx3vj9av        '].
1590975497.0369272 [planner_server-71] [INFO] [global_costmap.global_costmap_rclcpp_node]: Message Filter dropping message: frame 'base_scan_gt' at time 8.600 for reason 'Unknown'
1590975500.7047789 [planner_server-71] [INFO] [global_costmap.global_costmap_rclcpp_node]: Message Filter dropping message: frame 'base_scan_gt' at time 24.000 for reason 'Unknown'
...
991 similar lines
...
1590995577.4389014 [controller_server-70] [INFO] [local_costmap.local_costmap_rclcpp_node]: Message Filter dropping message: frame 'base_scan_gt' at time 18638.729 for reason 'Unknown'
1590995595.2603343 [planner_server-71] [INFO] [global_costmap.global_costmap_rclcpp_node]: Message Filter dropping message: frame 'base_scan_gt' at time 18286.526 for reason 'Unknown'
1590995683.7506635 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
1590995683.7553263 [waypoint_follower-74] [INFO] [rclcpp]: signal_handler(signal_value=2)
1590995683.7565181 [static_transform_publisher-68] [INFO] [rclcpp]: signal_handler(signal_value=2)
1590995683.7638111 [planner_server-71] [INFO] [rclcpp]: signal_handler(signal_value=2)
1590995683.7641566 [robot_state_publisher-67] [INFO] [rclcpp]: signal_handler(signal_value=2)
1590995683.7663212 [controller_server-70] [INFO] [rclcpp]: signal_handler(signal_value=2)
1590995683.7675335 [robot_state_publisher-69] [INFO] [rclcpp]: signal_handler(signal_value=2)
1590995683.7696064 [recoveries_server-72] [INFO] [rclcpp]: signal_handler(signal_value=2)
1590995683.7843037 [waypoint_follower-74] [INFO] [waypoint_follower]: Destroying
1590995683.8107643 [INFO] [recoveries_server-72]: process has finished cleanly [pid 6714]
1590995683.8156049 [INFO] [map_server-75]: process has finished cleanly [pid 6742]
1590995683.8472409 [INFO] [static_transform_publisher-68]: process has finished cleanly [pid 6692]
1590995683.8622200 [INFO] [robot_state_publisher-69]: process has finished cleanly [pid 6693]
1590995683.8651915 [INFO] [robot_state_publisher-67]: process has finished cleanly [pid 6691]
1590995683.8747897 [INFO] [controller_server-70]: process has finished cleanly [pid 6695]
1590995683.8750751 [INFO] [planner_server-71]: process has finished cleanly [pid 6707]
1590995683.9197950 [INFO] [waypoint_follower-74]: process has finished cleanly [pid 6724]
1590995683.9303401 [INFO] [lifecycle_manager-78]: process has finished cleanly [pid 6793]
1590995683.9864919 [INFO] [amcl-76]: process has finished cleanly [pid 6760]
1590995684.2813745 [INFO] [gzserver-66]: process has finished cleanly [pid 6690]
1590995686.6304350 [INFO] [localization_benchmark_supervisor-77]: process has finished cleanly [pid 6774]
```
