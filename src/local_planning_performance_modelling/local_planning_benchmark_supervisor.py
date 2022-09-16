# -*- coding: utf-8 -*-

import random
import time
import traceback

import networkx as nx
import numpy as np
import pandas as pd
import pyquaternion
import yaml

import rospy
import tf2_ros
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

import copy
import pickle
import psutil

import os
from os import path

from performance_modelling_py.environment import ground_truth_map
from performance_modelling_py.utils import print_info, print_error, print_fatal


class RunFailException(Exception):
    pass


def main():
    rospy.init_node('benchmark_supervisor', anonymous=False)

    node = None

    # noinspection PyBroadException
    try:
        node = LocalPlanningBenchmarkSupervisor()
        node.start_run()
        rospy.spin()

    except KeyboardInterrupt:
        node.ros_shutdown_callback()
    except RunFailException as e:
        print_error(e)
    except Exception:
        print_error("main: exception raised during rospy.spin:")
        print_error(traceback.format_exc())

    finally:
        if node is not None:
            node.end_run()
        if not rospy.is_shutdown():
            print_info("calling rospy signal_shutdown")
            rospy.signal_shutdown("run_terminated")


class LocalPlanningBenchmarkSupervisor:
    def __init__(self):

        # Debug variable
        self.prevent_shutdown = False  # Should be False, unless you are currently debugging. if True, runs will never end.

        # topics, services, actions, entities and frames names
        scan_topic = rospy.get_param('~scan_topic')
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic')
        odom_topic = rospy.get_param('~odom_topic')
        ground_truth_pose_topic = rospy.get_param('~ground_truth_pose_topic')
        estimated_pose_correction_topic = rospy.get_param('~estimated_pose_correction_topic')
        goal_pose_topic = rospy.get_param('~goal_pose_topic')
        self.navigate_to_pose_action = rospy.get_param('~navigate_to_pose_action')
        self.navigate_to_pose_topic = rospy.get_param('~navigate_to_pose_topic')
        self.fixed_frame = rospy.get_param('~fixed_frame')
        self.robot_base_frame = rospy.get_param('~robot_base_frame')
        self.robot_entity_name = rospy.get_param('~robot_entity_name')

        # file system paths
        self.run_output_folder = rospy.get_param('~run_output_folder')
        self.benchmark_data_folder = path.join(self.run_output_folder, "benchmark_data")
        self.ps_output_folder = path.join(self.benchmark_data_folder, "ps_snapshots")
        self.ground_truth_map_info_path = rospy.get_param('~ground_truth_map_info_path')

        # run parameters
        self.run_index = rospy.get_param('~run_index')
        self.run_timeout = rospy.get_param('~run_timeout')
        ps_snapshot_period = rospy.get_param('~ps_snapshot_period')
        write_estimated_poses_period = rospy.get_param('~write_estimated_poses_period')
        self.ps_pid_father = rospy.get_param('~pid_father')
        self.ps_processes = psutil.Process(self.ps_pid_father).children(recursive=True)  # list of processes children of the benchmark script, i.e., all ros nodes of the benchmark including this one
        self.ground_truth_map = ground_truth_map.GroundTruthMap(self.ground_truth_map_info_path)
        self.initial_pose_covariance_matrix = np.zeros((6, 6), dtype=float)
        self.initial_pose_covariance_matrix[0, 0] = rospy.get_param('~initial_pose_std_xy')**2
        self.initial_pose_covariance_matrix[1, 1] = rospy.get_param('~initial_pose_std_xy')**2
        self.initial_pose_covariance_matrix[5, 5] = rospy.get_param('~initial_pose_std_theta')**2
        self.goal_tolerance = rospy.get_param('~goal_tolerance')
        self.goal_obstacle_min_distance = rospy.get_param('~goal_obstacle_min_distance')
        self.goal_publication_type = rospy.get_param('~goal_publication_type')  

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
        rospy.Timer(rospy.Duration.from_sec(self.run_timeout), self.run_timeout_callback)
        rospy.Timer(rospy.Duration.from_sec(ps_snapshot_period), self.ps_snapshot_timer_callback)
        rospy.Timer(rospy.Duration.from_sec(write_estimated_poses_period), self.write_estimated_pose_timer_callback)

        # setup buffers
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # setup publishers
        self.goal_pose_publisher = rospy.Publisher(goal_pose_topic, PoseStamped, queue_size=1)
        self.navigate_to_pose_publisher = rospy.Publisher(self.navigate_to_pose_topic, PoseStamped, queue_size=1, latch=True)


        # setup subscribers
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber(estimated_pose_correction_topic, PoseWithCovarianceStamped, self.estimated_pose_correction_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(ground_truth_pose_topic, Odometry, self.ground_truth_pose_callback, queue_size=1)

        # setup action clients
        self.navigate_to_pose_action_client = SimpleActionClient(self.navigate_to_pose_action, MoveBaseAction)

    def start_run(self):
        print_info("waiting simulator and navigation stack", logger=rospy.loginfo)

        # wait to receive sensor data from the environment (e.g., a simulator may need time to startup)
        waiting_time = 0.0
        total_waiting_time = 0.0
        waiting_period = 0.5
        while not self.received_first_scan and not rospy.is_shutdown():
            time.sleep(waiting_period)
            waiting_time += waiting_period
            total_waiting_time += waiting_period
            if waiting_time > 5.0:
                rospy.logwarn('still waiting to receive first sensor message from environment')
                waiting_time = 0.0
            if total_waiting_time > 60.0:
                rospy.logfatal("timed out waiting to receive first sensor message from environment")
                raise RunFailException("timed out waiting to receive first sensor message from environment")
        print_info("finished waiting to receive first sensor message from environment", logger=rospy.loginfo)

        # get deleaved reduced Voronoi graph from ground truth map
        voronoi_graph = self.ground_truth_map.deleaved_reduced_voronoi_graph(minimum_radius=self.goal_obstacle_min_distance).copy()
        print_fatal(voronoi_graph.nodes)
        for i in voronoi_graph.nodes:
            print_info(i, voronoi_graph.nodes[i]['vertex'])
        

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
        self.goal_pose.header.stamp = rospy.Time.now()
        self.goal_pose.header.frame_id = self.fixed_frame
        self.goal_pose.pose = Pose()
        self.goal_pose.pose.position.x, self.goal_pose.pose.position.y = voronoi_graph.nodes[self.pseudo_random_voronoi_index]['vertex']
        q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=np.random.uniform(-np.pi, np.pi))
        self.goal_pose.pose.orientation = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        self.goal_pose_publisher.publish(self.goal_pose)

        self.write_event('run_start')
        self.run_started = True
        if self.goal_publication_type == 'action':
            self.send_action_goal() 
        else: 
            self.send_topic_goal()

    def send_action_goal(self):
        print_info("waiting for navigation action server ({n})".format(n=self.navigate_to_pose_action), logger=rospy.loginfo)
        action_client_timeout = 25.0
        action_client_warning_period = 5.0
        action_client_time_waited = 0.0
        while not self.navigate_to_pose_action_client.wait_for_server(timeout=rospy.Duration.from_sec(action_client_warning_period)):
            action_client_time_waited += action_client_warning_period
            if action_client_time_waited >= action_client_timeout:
                self.write_event('failed_to_communicate_with_navigation_node')
                raise RunFailException("navigate_to_pose action server not available")
            else:
                rospy.logwarn("still waiting for navigation action server ({n})".format(n=self.navigate_to_pose_action))
        print_info("navigation action server active ({n})".format(n=self.navigate_to_pose_action), logger=rospy.loginfo)

        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = self.fixed_frame
        goal_msg.target_pose.pose = self.goal_pose.pose

        self.navigate_to_pose_action_client.send_goal(goal_msg)
        self.write_event('navigation_goal_sent')        # viene usato per capire se la run è partita  
        self.write_event('navigation_goal_accepted')

        self.run_data["goal_pose"] = self.goal_pose.pose
        self.run_data["voronoi_node_index"] = self.pseudo_random_voronoi_index

        if not self.navigate_to_pose_action_client.wait_for_result(timeout=rospy.Duration.from_sec(self.run_timeout)):
            self.write_event('waypoint_timeout')
            self.write_event('supervisor_finished')
            raise RunFailException("waypoint_timeout")

        self.run_data["navigation_final_status"] = self.navigate_to_pose_action_client.get_state()
        if self.navigate_to_pose_action_client.get_state() == GoalStatus.SUCCEEDED:
            self.write_event('navigation_succeeded')
            goal_position = self.goal_pose.pose.position
            if self.latest_estimated_position is not None:
                current_position = self.latest_estimated_position
                distance_from_goal = np.sqrt((goal_position.x - current_position.x) ** 2 + (goal_position.y - current_position.y) ** 2)
                if distance_from_goal < self.goal_tolerance:
                    self.write_event('navigation_goal_reached')
                else:
                    rospy.logerr("goal status succeeded but current position farther from goal position than tolerance")
                    self.write_event('navigation_goal_not_reached')
            else:
                rospy.logfatal("estimated position not set")
                self.write_event('estimated_position_not_received')
                if not self.prevent_shutdown:
                    rospy.signal_shutdown("estimated position not set")
        else:
            print_info('navigation action failed with status {}, {}'.format(self.navigate_to_pose_action_client.get_state(), self.navigate_to_pose_action_client.get_goal_status_text()), logger=rospy.loginfo)
            self.write_event('navigation_failed')

        self.write_event('run_completed')
        if not self.prevent_shutdown:
            rospy.signal_shutdown('run completed')

    def send_topic_goal(self):    
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = self.fixed_frame
        goal_msg.pose = self.goal_pose.pose
        
        self.navigate_to_pose_publisher.publish(goal_msg)
        self.write_event('navigation_goal_sent')        #viene usato per capire se la run è partita  
        self.write_event('navigation_goal_accepted')

        self.run_data["goal_pose"] = self.goal_pose.pose
        self.run_data["voronoi_node_index"] = self.pseudo_random_voronoi_index
            
        goal_position = self.goal_pose.pose.position
        start_time = rospy.Time.now()
        current_position = None
        last_time_robot_movement = None
        last_time_robot_stuck = None
        start = start_time

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            rospy.sleep(0.1)
            total_waiting_time = current_time - start_time
            latest_position = current_position      # la prima volta è uguale a None
            if total_waiting_time.to_sec() > self.run_timeout:
                self.write_event('waypoint_timeout')
                self.write_event('supervisor_finished')
                raise RunFailException("waypoint_timeout")
                break   
            if self.latest_estimated_position is not None:                
                current_position = self.latest_estimated_position
                distance_from_goal = np.sqrt((goal_position.x - current_position.x) ** 2 + (goal_position.y - current_position.y) ** 2)
                if distance_from_goal < self.goal_tolerance:
                    self.write_event('navigation_succeeded')
                    self.write_event('navigation_goal_reached')
                    break
                else: 
                    rospy.loginfo("attempting to reach goal position")

                if latest_position is not None:
                    # distance between current position and latest saved position of the robot
                    latest_distance = np.sqrt((latest_position.x - current_position.x) ** 2 + (latest_position.y - current_position.y) ** 2)
                    print_error("Latest distance:")
                    print_error(latest_distance)
                    if latest_distance > 0.009: # il robot si sta muovendo
                        time_stuck = 0
                        start = current_time
                        print_error("robot is moving")
                        last_time_robot_movement = rospy.Time.now() # ultimo istante in cui il robot si è mosso
                    else:    # il robot non si sta muovendo
                        print_error("robot is NOT moving")
                        last_time_robot_stuck = rospy.Time.now() # ultimo istante in cui il robot è stato rilevato essere fermo
                    
                    # if last_time_robot_stuck is not None:
                    #     time_stuck = last_time_robot_stuck - start
                    #     print_error("Time Stuck:")
                    #     print_error(time_stuck.to_sec()) 


                    if last_time_robot_movement is not None:    # questo funziona se il robot si è mosso all'inizio e poi si blocca
                        total_elapsed_time = current_time - last_time_robot_movement
                        print_error("Time 2:")
                        print_error(total_elapsed_time.to_sec()) 
                        if total_elapsed_time.to_sec() > 10:    # vel_max = 23 cm/s           50 cm / 23 cm /s = 2 s, per ora settato a 10s, per le run si può mettere a 60s
                            self.write_event('robot_stuck')
                            raise RunFailException('robot_stuck')
                            break
                       
                    #TODO mettere le threshold 0.1 e 20 come parametri 
                 

        self.write_event('run_completed')
        if not self.prevent_shutdown:
            rospy.signal_shutdown('run completed')

    def ros_shutdown_callback(self):
        """
        This function is called when the node receives an interrupt signal (KeyboardInterrupt).
        """
        print_info("asked to shutdown, terminating run", logger=rospy.loginfo)
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

    def run_timeout_callback(self, _):
        print_error("terminating supervisor due to timeout, terminating run")
        self.write_event('run_timeout')
        self.write_event('supervisor_finished')
        rospy.signal_shutdown("run_timeout")

    def scan_callback(self, laser_scan_msg):
        self.received_first_scan = True
        #if self.goal_publication_type == 'topic': return
        if not self.run_started:
            return

        msg_time = laser_scan_msg.header.stamp.to_sec()
        with open(self.scans_file_path, 'a') as scans_file:
            scans_file.write("{t}, {angle_min}, {angle_max}, {angle_increment}, {range_min}, {range_max}, {ranges}\n".format(
                t=msg_time,
                angle_min=laser_scan_msg.angle_min,
                angle_max=laser_scan_msg.angle_max,
                angle_increment=laser_scan_msg.angle_increment,
                range_min=laser_scan_msg.range_min,
                range_max=laser_scan_msg.range_max,
                ranges=', '.join(map(str, laser_scan_msg.ranges))))

    def cmd_vel_callback(self, twist_msg):
        #if self.goal_publication_type == 'topic': return
        if not self.run_started:
            return

        self.cmd_vel_df = self.cmd_vel_df.append({
            't': rospy.Time.now().to_sec(),
            'linear_x': twist_msg.linear.x,
            'linear_y': twist_msg.linear.y,
            'linear_z': twist_msg.linear.z,
            'angular_x': twist_msg.angular.x,
            'angular_y': twist_msg.angular.y,
            'angular_z': twist_msg.angular.z,
        }, ignore_index=True)

    def write_estimated_pose_timer_callback(self, _):
        try:
            transform_msg = self.tf_buffer.lookup_transform(self.fixed_frame, self.robot_base_frame, rospy.Time())
            self.latest_estimated_position = transform_msg.transform.translation  # save the latest position to check if the robot has reached the goal within tolerance
            orientation = transform_msg.transform.rotation
            theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll

            self.estimated_poses_df = self.estimated_poses_df.append({
                't': transform_msg.header.stamp.to_sec(),
                'x': transform_msg.transform.translation.x,
                'y': transform_msg.transform.translation.y,
                'theta': theta
            }, ignore_index=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def estimated_pose_correction_callback(self, pose_with_covariance_msg):
        if not self.run_started:
            return

        orientation = pose_with_covariance_msg.pose.pose.orientation
        theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll
        covariance_mat = np.array(pose_with_covariance_msg.pose.covariance).reshape(6, 6)

        self.estimated_correction_poses_df = self.estimated_correction_poses_df.append({
            't': pose_with_covariance_msg.header.stamp.to_sec(),
            'x': pose_with_covariance_msg.pose.pose.position.x,
            'y': pose_with_covariance_msg.pose.pose.position.y,
            'theta': theta,
            'cov_x_x': covariance_mat[0, 0],
            'cov_x_y': covariance_mat[0, 1],
            'cov_y_y': covariance_mat[1, 1],
            'cov_theta_theta': covariance_mat[5, 5]
        }, ignore_index=True)

    def odom_callback(self, odometry_msg):
        #if self.goal_publication_type == 'topic': return
        if not self.run_started:
            return

        orientation = odometry_msg.pose.pose.orientation
        theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll

        self.odom_df = self.odom_df.append({
            't': odometry_msg.header.stamp.to_sec(),
            'x': odometry_msg.pose.pose.position.x,
            'y': odometry_msg.pose.pose.position.y,
            'theta': theta,
            'v_x': odometry_msg.twist.twist.linear.x,
            'v_y': odometry_msg.twist.twist.linear.y,
            'v_theta': odometry_msg.twist.twist.angular.z,
        }, ignore_index=True)

    def ground_truth_pose_callback(self, odometry_msg):
        if not self.run_started:
            return

        orientation = odometry_msg.pose.pose.orientation
        theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll

        self.ground_truth_poses_df = self.ground_truth_poses_df.append({
            't': odometry_msg.header.stamp.to_sec(),
            'x': odometry_msg.pose.pose.position.x,
            'y': odometry_msg.pose.pose.position.y,
            'theta': theta,
            'v_x': odometry_msg.twist.twist.linear.x,
            'v_y': odometry_msg.twist.twist.linear.y,
            'v_theta': odometry_msg.twist.twist.angular.z,
        }, ignore_index=True)

    def ps_snapshot_timer_callback(self, _):
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
                process_copy['rostime_stamp'] = rospy.Time.now().to_sec()

                processes_dicts_list.append(process_copy)
            except KeyError:
                pass
        try:
            with open(ps_snapshot_file_path, 'wb') as ps_snapshot_file:
                pickle.dump(processes_dicts_list, ps_snapshot_file)
        except TypeError:
            rospy.logerr(traceback.format_exc())

        self.ps_snapshot_count += 1

    def init_run_events_file(self):
        try:
            with open(self.run_events_file_path, 'w') as run_events_file:
                run_events_file.write("t, real_time, event\n")
        except IOError as e:
            rospy.logerr("benchmark_supervisor.init_event_file: could not write header to run_events_file")
            rospy.logerr(e)

    def write_event(self, event):
        ros_time = rospy.Time.now().to_sec()
        real_time = time.time()
        event_string = "t: {ros_time}, real_time: {real_time}, event: {event}".format(ros_time=ros_time, real_time=real_time, event=str(event))
        event_csv_line = "{ros_time}, {real_time}, {event}\n".format(ros_time=ros_time, real_time=real_time, event=str(event))
        print_info(event_string, logger=rospy.loginfo)
        try:
            with open(self.run_events_file_path, 'a') as run_events_file:
                run_events_file.write(event_csv_line)
        except IOError as e:
            rospy.logerr("write_event: could not write event to run_events_file: {event_string}".format(event_string=event_string))
            rospy.logerr(e)

