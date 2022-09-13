# -*- coding: utf-8 -*-

import random
import time
import traceback

import geometry_msgs
import lifecycle_msgs
import nav_msgs
import networkx as nx
import numpy as np
import pandas as pd
import pyquaternion
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import SetEntityState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from performance_modelling_py.environment import ground_truth_map
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import TransitionEvent
from rclpy.qos import qos_profile_sensor_data, qos_profile_parameter_events
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

import tf2_ros

import copy
import pickle
import psutil

import os
from os import path

from performance_modelling_py.utils import print_info, print_error, nanoseconds_to_seconds
from std_srvs.srv import Empty


class RunFailException(Exception):
    pass


def main(args=None):
    rclpy.init(args=args)

    node = None

    # noinspection PyBroadException
    try:
        node = LocalPlanningBenchmarkSupervisor()
        node.start_run()
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.ros_shutdown_callback()
    except RunFailException as e:
        print_error(e)
    except Exception:
        print_error("main: exception raised during rclpy.spin:")
        print_error(traceback.format_exc())

    finally:
        if node is not None:
            node.end_run()


class LocalPlanningBenchmarkSupervisor(Node):
    def __init__(self):
        super().__init__('local_planning_benchmark_supervisor', automatically_declare_parameters_from_overrides=True)

        # Debug variable
        self.prevent_shutdown = False  # Should be False, unless you are currently debugging. if True, runs will never end.

        # topics, services, actions, entities and frames names
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        estimated_pose_correction_topic = self.get_parameter('estimated_pose_correction_topic').value
        goal_pose_topic = self.get_parameter('goal_pose_topic').value
        navigation_transition_event_topic = self.get_parameter('navigation_transition_event_topic').value
        lifecycle_manager_service = self.get_parameter('lifecycle_manager_service').value
        global_costmap_get_parameters_service = self.get_parameter('global_costmap_get_parameters_service').value
        pause_physics_service = self.get_parameter('pause_physics_service').value
        unpause_physics_service = self.get_parameter('unpause_physics_service').value
        set_entity_state_service = self.get_parameter('set_entity_state_service').value
        navigate_to_pose_action = self.get_parameter('navigate_to_pose_action').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_entity_name = self.get_parameter('robot_entity_name').value

        # file system paths
        self.run_output_folder = self.get_parameter('run_output_folder').value
        self.benchmark_data_folder = path.join(self.run_output_folder, "benchmark_data")
        self.ps_output_folder = path.join(self.benchmark_data_folder, "ps_snapshots")
        self.ground_truth_map_info_path = self.get_parameter("ground_truth_map_info_path").value

        # run parameters
        self.run_index = self.get_parameter('run_index').value
        run_timeout = self.get_parameter('run_timeout').value
        ps_snapshot_period = self.get_parameter('ps_snapshot_period').value
        write_estimated_poses_period = self.get_parameter('write_estimated_poses_period').value
        self.ps_pid_father = self.get_parameter('pid_father').value
        self.ps_processes = psutil.Process(self.ps_pid_father).children(recursive=True)  # list of processes children of the benchmark script, i.e., all ros nodes of the benchmark including this one
        self.ground_truth_map = ground_truth_map.GroundTruthMap(self.ground_truth_map_info_path)
        self.initial_pose_covariance_matrix = np.zeros((6, 6), dtype=float)
        self.initial_pose_covariance_matrix[0, 0] = self.get_parameter('initial_pose_std_xy').value**2
        self.initial_pose_covariance_matrix[1, 1] = self.get_parameter('initial_pose_std_xy').value**2
        self.initial_pose_covariance_matrix[5, 5] = self.get_parameter('initial_pose_std_theta').value**2
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_obstacle_min_distance = self.get_parameter('goal_obstacle_min_distance').value

        # run variables
        self.run_started = False
        self.terminate = False
        self.ps_snapshot_count = 0
        self.received_first_scan = False
        self.latest_estimated_position = None
        self.navigation_node_activated = False
        self.pseudo_random_voronoi_index = None
        self.goal_pose = None

        # prepare folder structure
        if not path.exists(self.benchmark_data_folder):
            os.makedirs(self.benchmark_data_folder)

        if not path.exists(self.ps_output_folder):
            os.makedirs(self.ps_output_folder)

        # file paths for benchmark data
        self.estimated_poses_file_path = path.join(self.benchmark_data_folder, "estimated_poses.csv")
        self.estimated_correction_poses_file_path = path.join(self.benchmark_data_folder, "estimated_correction_poses.csv")
        self.odom_file_path = path.join(self.benchmark_data_folder, "odom.csv")
        self.ground_truth_poses_file_path = path.join(self.benchmark_data_folder, "ground_truth_poses.csv")
        self.cmd_vel_file_path = path.join(self.benchmark_data_folder, "cmd_vel.csv")
        self.scans_file_path = path.join(self.benchmark_data_folder, "scans.csv")
        self.run_events_file_path = path.join(self.benchmark_data_folder, "run_events.csv")
        self.run_data_file_path = path.join(self.benchmark_data_folder, "run_data.yaml")
        self.init_run_events_file()

        # pandas dataframes for benchmark data
        self.estimated_poses_df = pd.DataFrame(columns=['t', 'x', 'y', 'theta'])
        self.estimated_correction_poses_df = pd.DataFrame(columns=['t', 'x', 'y', 'theta', 'cov_x_x', 'cov_x_y', 'cov_y_y', 'cov_theta_theta'])
        self.odom_df = pd.DataFrame(columns=['t', 'x', 'y', 'theta', 'v_x', 'v_y', 'v_theta'])
        self.ground_truth_poses_df = pd.DataFrame(columns=['t', 'x', 'y', 'theta', 'v_x', 'v_y', 'v_theta'])
        self.cmd_vel_df = pd.DataFrame(columns=['t', 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'])
        self.run_data = dict()

        # setup timers
        self.create_timer(run_timeout, self.run_timeout_callback)
        self.create_timer(ps_snapshot_period, self.ps_snapshot_timer_callback)
        self.create_timer(write_estimated_poses_period, self.write_estimated_pose_timer_callback)

        # setup service clients
        self.lifecycle_manager_service_client = self.create_client(ManageLifecycleNodes, lifecycle_manager_service)
        self.global_costmap_get_parameters_service_client = self.create_client(GetParameters, global_costmap_get_parameters_service)
        self.pause_physics_service_client = self.create_client(Empty, pause_physics_service)
        self.unpause_physics_service_client = self.create_client(Empty, unpause_physics_service)
        self.set_entity_state_service_client = self.create_client(SetEntityState, set_entity_state_service)

        # setup buffers
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # setup publishers
        self.goal_pose_publisher = self.create_publisher(PoseStamped, goal_pose_topic, 1)

        # setup subscribers
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, estimated_pose_correction_topic, self.estimated_pose_correction_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, ground_truth_pose_topic, self.ground_truth_pose_callback, qos_profile_sensor_data)
        self.create_subscription(TransitionEvent, navigation_transition_event_topic, self.lifecycle_transition_event_callback, qos_profile_parameter_events)

        # setup action clients
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, navigate_to_pose_action)
        self.navigate_to_pose_action_goal_future = None
        self.navigate_to_pose_action_result_future = None

    def start_run(self):
        print_info("waiting simulator and navigation stack", logger=self.get_logger().info)

        # wait to receive sensor data from the environment (e.g., a simulator may need time to startup) and for the navigation stack to activate
        waiting_time = 0.0
        total_waiting_time = 0.0
        waiting_period = 0.5
        while not self.received_first_scan or not self.navigation_node_activated:
            if not rclpy.ok():
                self.get_logger().fatal("ros shutdown while waiting to receive first sensor message from environment and navigation to be activated")
                raise RunFailException("ros shutdown while waiting to receive first sensor message from environment and navigation to be activated")

            time.sleep(waiting_period)
            rclpy.spin_once(self)
            waiting_time += waiting_period
            total_waiting_time += waiting_period
            if waiting_time > 5.0:
                self.get_logger().warning("still waiting to receive first sensor message from environment and navigation to be activated")
                waiting_time = 0.0
            if total_waiting_time > 60.0:
                self.get_logger().fatal("timed out waiting to receive first sensor message from environment and navigation to be activated")
                raise RunFailException("timed out waiting to receive first sensor message from environment and navigation to be activated")
        print_info("finished waiting to receive first sensor message from environment and navigation to be activated", logger=self.get_logger().info)

        # get deleaved reduced Voronoi graph from ground truth map
        voronoi_graph = self.ground_truth_map.deleaved_reduced_voronoi_graph(minimum_radius=self.goal_obstacle_min_distance).copy()

        # in case the graph has multiple unconnected components, remove the components with less than two nodes
        too_small_voronoi_graph_components = list(filter(lambda component: len(component) < 2, nx.connected_components(voronoi_graph)))

        for graph_component in too_small_voronoi_graph_components:
            voronoi_graph.remove_nodes_from(graph_component)

        if len(voronoi_graph.nodes) < 2:
            self.write_event('insufficient_number_of_nodes_in_deleaved_reduced_voronoi_graph')
            raise RunFailException("insufficient number of nodes in deleaved_reduced_voronoi_graph, can not generate traversal path")

        # select the node pseudo-randomly using the run number
        # the list of indices is always shuffled the same way (seed = 0), so each run number will always correspond to the same Voronoi node
        nil = copy.copy(list(voronoi_graph.nodes))  # list of the indices of the nodes in voronoi_graph.nodes
        random.Random(0).shuffle(nil)
        self.pseudo_random_voronoi_index = nil[self.run_index % len(nil)]

        # convert Voronoi node to pose
        self.goal_pose = PoseStamped()
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.header.frame_id = self.fixed_frame
        self.goal_pose.pose = Pose()
        self.goal_pose.pose.position.x, self.goal_pose.pose.position.y = voronoi_graph.nodes[self.pseudo_random_voronoi_index]['vertex']
        q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=np.random.uniform(-np.pi, np.pi))
        self.goal_pose.pose.orientation = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        self.goal_pose_publisher.publish(self.goal_pose)

        self.write_event('run_start')
        self.run_started = True
        self.send_goal()

    def send_goal(self):

        if not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=5.0):
            self.write_event('failed_to_communicate_with_navigation_node')
            raise RunFailException("navigate_to_pose action server not available")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.navigate_to_pose_action_goal_future = self.navigate_to_pose_action_client.send_goal_async(goal_msg)
        self.navigate_to_pose_action_goal_future.add_done_callback(self.goal_response_callback)
        self.write_event('navigation_goal_sent')
        self.run_data["goal_pose"] = self.goal_pose.pose
        self.run_data["voronoi_node_index"] = self.pseudo_random_voronoi_index

    def ros_shutdown_callback(self):
        """
        This function is called when the node receives an interrupt signal (KeyboardInterrupt).
        """
        print_info("asked to shutdown, terminating run", logger=self.get_logger().info)
        self.write_event('ros_shutdown')
        self.write_event('supervisor_finished')

    def end_run(self):
        """
        This function is called after the run has completed, whether the run finished correctly, or there was an exception.
        The only case in which this function is not called is if an exception was raised from self.__init__
        """
        self.estimated_poses_df.to_csv(self.estimated_poses_file_path, index=False)
        self.estimated_correction_poses_df.to_csv(self.estimated_correction_poses_file_path, index=False)
        self.odom_df.to_csv(self.odom_file_path, index=False)
        self.ground_truth_poses_df.to_csv(self.ground_truth_poses_file_path, index=False)
        self.cmd_vel_df.to_csv(self.cmd_vel_file_path, index=False)

        with open(self.run_data_file_path, 'w') as run_data_file:
            yaml.dump(self.run_data, run_data_file)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # if the goal is rejected try with the next goal
        if not goal_handle.accepted:
            self.get_logger().error('navigation action goal rejected')
            self.write_event('navigation_goal_rejected')
            if not self.prevent_shutdown:
                rclpy.shutdown()
            return

        self.write_event('navigation_goal_accepted')

        self.navigate_to_pose_action_result_future = goal_handle.get_result_async()
        self.navigate_to_pose_action_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.run_data["navigation_final_status"] = status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.write_event('navigation_succeeded')
            goal_position = self.goal_pose.pose.position
            if self.latest_estimated_position is not None:
                current_position = self.latest_estimated_position
                distance_from_goal = np.sqrt((goal_position.x - current_position.x) ** 2 + (goal_position.y - current_position.y) ** 2)
                if distance_from_goal < self.goal_tolerance:
                    self.write_event('navigation_goal_reached')
                else:
                    self.get_logger().error("goal status succeeded but current position farther from goal position than tolerance")
                    self.write_event('navigation_goal_not_reached')
            else:
                self.get_logger().fatal("estimated position not set")
                self.write_event('estimated_position_not_received')
                if not self.prevent_shutdown:
                    rclpy.shutdown()
        else:
            print_info('navigation action failed with status {}'.format(status), logger=self.get_logger().info)
            self.write_event('navigation_failed')

        self.write_event('run_completed')
        if not self.prevent_shutdown:
            rclpy.shutdown()

    def lifecycle_transition_event_callback(self, transition_event_msg: lifecycle_msgs.msg.TransitionEvent):
        # send the initial pose as soon as the localization node activates the first time
        if transition_event_msg.goal_state.label == 'active' and not self.navigation_node_activated:
            self.navigation_node_activated = True
            self.write_event("navigation_node_activated")

    def run_timeout_callback(self):
        self.get_logger().error("terminating supervisor due to timeout, terminating run")
        self.write_event('run_timeout')
        if not self.prevent_shutdown:
            rclpy.shutdown()
            self.get_logger().info("called rclpy.shutdown()")
            # self.destroy_node()

    def scan_callback(self, laser_scan_msg):
        self.received_first_scan = True
        if not self.run_started:
            return

        msg_time = nanoseconds_to_seconds(laser_scan_msg.header.stamp.nanosec) + float(laser_scan_msg.header.stamp.sec)
        with open(self.scans_file_path, 'a') as scans_file:
            scans_file.write("{t}, {angle_min}, {angle_max}, {angle_increment}, {range_min}, {range_max}, {ranges}\n".format(
                t=msg_time,
                angle_min=laser_scan_msg.angle_min,
                angle_max=laser_scan_msg.angle_max,
                angle_increment=laser_scan_msg.angle_increment,
                range_min=laser_scan_msg.range_min,
                range_max=laser_scan_msg.range_max,
                ranges=', '.join(map(str, laser_scan_msg.ranges))))

    def cmd_vel_callback(self, twist_msg: Twist):
        if not self.run_started:
            return

        self.cmd_vel_df = pd.concat([
            self.cmd_vel_df,
            pd.DataFrame({
                't': [nanoseconds_to_seconds(self.get_clock().now().nanoseconds)],
                'linear_x': [twist_msg.linear.x],
                'linear_y': [twist_msg.linear.y],
                'linear_z': [twist_msg.linear.z],
                'angular_x': [twist_msg.angular.x],
                'angular_y': [twist_msg.angular.y],
                'angular_z': [twist_msg.angular.z],
            })], ignore_index=True)

    def write_estimated_pose_timer_callback(self):
        try:
            transform_msg = self.tf_buffer.lookup_transform(self.fixed_frame, self.robot_base_frame, Time())
            self.latest_estimated_position = transform_msg.transform.translation  # save the latest position to check if the robot has reached the goal within tolerance
            orientation = transform_msg.transform.rotation
            theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll

            self.estimated_poses_df = pd.concat([
                self.estimated_poses_df,
                pd.DataFrame({
                    't': [nanoseconds_to_seconds(Time.from_msg(transform_msg.header.stamp).nanoseconds)],
                    'x': [transform_msg.transform.translation.x],
                    'y': [transform_msg.transform.translation.y],
                    'theta': [theta],
                })], ignore_index=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def estimated_pose_correction_callback(self, pose_with_covariance_msg: geometry_msgs.msg.PoseWithCovarianceStamped):
        if not self.run_started:
            return

        orientation = pose_with_covariance_msg.pose.pose.orientation
        theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll
        covariance_mat = np.array(pose_with_covariance_msg.pose.covariance).reshape(6, 6)

        self.estimated_correction_poses_df = pd.concat([
            self.estimated_correction_poses_df,
            pd.DataFrame({
                't': [nanoseconds_to_seconds(Time.from_msg(pose_with_covariance_msg.header.stamp).nanoseconds)],
                'x': [pose_with_covariance_msg.pose.pose.position.x],
                'y': [pose_with_covariance_msg.pose.pose.position.y],
                'theta': [theta],
                'cov_x_x': [covariance_mat[0, 0]],
                'cov_x_y': [covariance_mat[0, 1]],
                'cov_y_y': [covariance_mat[1, 1]],
                'cov_theta_theta': [covariance_mat[5, 5]],
            })], ignore_index=True)

    def odom_callback(self, odometry_msg: nav_msgs.msg.Odometry):
        if not self.run_started:
            return

        orientation = odometry_msg.pose.pose.orientation
        theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll

        self.odom_df = pd.concat([
            self.odom_df,
            pd.DataFrame({
                't': [nanoseconds_to_seconds(Time.from_msg(odometry_msg.header.stamp).nanoseconds)],
                'x': [odometry_msg.pose.pose.position.x],
                'y': [odometry_msg.pose.pose.position.y],
                'theta': [theta],
                'v_x': [odometry_msg.twist.twist.linear.x],
                'v_y': [odometry_msg.twist.twist.linear.y],
                'v_theta': [odometry_msg.twist.twist.angular.z],
            })], ignore_index=True)

    def ground_truth_pose_callback(self, odometry_msg: nav_msgs.msg.Odometry):
        if not self.run_started:
            return

        orientation = odometry_msg.pose.pose.orientation
        theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll

        self.ground_truth_poses_df = pd.concat([
            self.ground_truth_poses_df,
            pd.DataFrame({
                't': [nanoseconds_to_seconds(Time.from_msg(odometry_msg.header.stamp).nanoseconds)],
                'x': [odometry_msg.pose.pose.position.x],
                'y': [odometry_msg.pose.pose.position.y],
                'theta': [theta],
                'v_x': [odometry_msg.twist.twist.linear.x],
                'v_y': [odometry_msg.twist.twist.linear.y],
                'v_theta': [odometry_msg.twist.twist.angular.z],
            })], ignore_index=True)

    def ps_snapshot_timer_callback(self):
        ps_snapshot_file_path = path.join(self.ps_output_folder, "ps_{i:08d}.pkl".format(i=self.ps_snapshot_count))

        processes_dicts_list = list()
        for process in self.ps_processes:
            try:
                process_copy = copy.deepcopy(process.as_dict())  # get all information about the process
            except psutil.NoSuchProcess:  # processes may have died, causing this exception to be raised from psutil.Process.as_dict
                continue
            try:
                # delete uninteresting values and add time information
                del process_copy['connections']
                del process_copy['memory_maps']
                del process_copy['environ']
                process_copy['realtime_stamp'] = time.time()
                process_copy['rostime_stamp'] = nanoseconds_to_seconds(self.get_clock().now().nanoseconds)

                processes_dicts_list.append(process_copy)
            except KeyError:
                pass
        try:
            with open(ps_snapshot_file_path, 'wb') as ps_snapshot_file:
                pickle.dump(processes_dicts_list, ps_snapshot_file)
        except TypeError:
            self.get_logger().error(traceback.format_exc())

        self.ps_snapshot_count += 1

    def init_run_events_file(self):
        try:
            with open(self.run_events_file_path, 'w') as run_events_file:
                run_events_file.write("t, real_time, event\n")
        except IOError as e:
            self.get_logger().error("slam_benchmark_supervisor.init_event_file: could not write header to run_events_file")
            self.get_logger().error(e)

    def write_event(self, event):
        ros_time = nanoseconds_to_seconds(self.get_clock().now().nanoseconds)
        real_time = time.time()
        event_string = f"t: {ros_time}, real_time: {real_time}, event: {str(event)}"
        event_csv_line = f"{ros_time}, {real_time}, {str(event)}\n"
        print_info(event_string, logger=self.get_logger().info)
        try:
            with open(self.run_events_file_path, 'a') as run_events_file:
                run_events_file.write(event_csv_line)
        except IOError as e:
            self.get_logger().error(f"write_event: could not write event to run_events_file: {event_string}")
            self.get_logger().error(e)

    def call_service(self, service_client, request, fail_timeout=30.0, warning_timeout=5.0):
        time_waited = 0.0
        while not service_client.wait_for_service(timeout_sec=warning_timeout) and rclpy.ok():
            self.get_logger().warning(f'supervisor: still waiting {service_client.srv_name} to become available')
            time_waited += warning_timeout
            if time_waited >= fail_timeout:
                self.get_logger().error(f'{service_client.srv_name} was not available')
                raise RunFailException(f"{service_client.srv_name} was not available")

        srv_future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self, srv_future)
        return srv_future.result()
