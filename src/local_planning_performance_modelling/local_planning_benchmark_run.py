# -*- coding: utf-8 -*-

from __future__ import print_function
from cmath import inf

import os
from platform import node
import shutil

from numpy import size

import rospy
import yaml
import networkx as nx
import copy
import random
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
import pyquaternion

import xml.etree.ElementTree as et
from xml.etree.ElementTree import Element

import time
from os import path
import numpy as np
from math import sqrt

from performance_modelling_py.benchmark_execution.log_software_versions import log_packages_and_repos
from performance_modelling_py.utils import backup_file_if_exists, print_info, print_error, print_fatal
from performance_modelling_py.component_proxies.ros1_component import Component
from performance_modelling_py.environment import ground_truth_map



class BenchmarkRun(object):
    def __init__(self, run_id, run_output_folder, benchmark_log_path, parameters_combination_dict, benchmark_configuration_dict, args_parser):

        # run configuration
        self.run_id = run_id
        self.run_output_folder = run_output_folder
        self.benchmark_log_path = benchmark_log_path
        self.run_parameters = parameters_combination_dict
        self.benchmark_configuration = benchmark_configuration_dict
        self.launch_rviz = args_parser.rviz
        self.launch_gzclient = args_parser.gzclient
        self.do_not_record = args_parser.do_not_record
        self.use_sim_time = True

        # environment parameters
        robots_dataset_folder = path.expanduser(self.benchmark_configuration['robots_dataset'])

        self.environment_folder = path.join(path.expanduser(self.benchmark_configuration['environments_dataset']), self.run_parameters['environment_name'])
        self.scene_file_path = path.join(self.environment_folder, "data", "scene.xml")
        self.map_info_file_path = path.join(self.environment_folder, "data", "map.yaml")
        self.gazebo_model_path_env_var = ":".join(map(
            lambda p: path.expanduser(p),
            self.benchmark_configuration['gazebo_model_path_env_var'] + [path.dirname(path.abspath(self.environment_folder)), self.run_output_folder]
        ))
        self.gazebo_plugin_path_env_var = ":".join(map(
            lambda p: path.expanduser(p),
            self.benchmark_configuration['gazebo_plugin_path_env_var']
        ))

        self.run_index = self.run_parameters['run_index']
        robot_model = self.run_parameters['robot_model']
        self.localization_node = self.run_parameters['localization_node']
        local_planner_node = self.run_parameters['local_planner_node']
        global_planner_node = self.run_parameters['global_planner_node']
        max_steering_angle_deg = self.run_parameters['max_steering_angle_deg'] if 'max_steering_angle_deg' in self.run_parameters else None
        max_steering_rad = (max_steering_angle_deg/180.0) * np.pi if 'max_steering_angle_deg' in self.run_parameters else None
        alpha_1, alpha_2, alpha_3, alpha_4 = self.run_parameters['odometry_error']
        pedestrian_number = self.run_parameters['pedestrian_number']

        amcl_alpha_1, amcl_alpha_2, amcl_alpha_3, amcl_alpha_4 = [None]*4
        localization_generator_update_rate = None
        localization_generator_translation_error = None
        localization_generator_rotation_error = None
        localization_generator_normalized_relative_translation_error = None
        localization_generator_normalized_relative_rotation_error = None
        if self.localization_node == 'amcl':
            amcl_alpha_factor = self.run_parameters['amcl_alpha_factor']
            if alpha_1 == 0 and alpha_2 == 0 and alpha_3 == 0 and alpha_4 == 0:
                amcl_alpha_1, amcl_alpha_2, amcl_alpha_3, amcl_alpha_4 = self.benchmark_configuration['amcl_ground_truth_alpha']
            else:
                amcl_alpha_1, amcl_alpha_2, amcl_alpha_3, amcl_alpha_4 = amcl_alpha_factor * alpha_1, amcl_alpha_factor * alpha_2, amcl_alpha_factor * alpha_3, amcl_alpha_factor * alpha_4
        elif self.localization_node == 'localization_generator':
            localization_generator_update_rate = self.run_parameters['localization_generator_update_rate']
            localization_generator_translation_error = self.run_parameters['localization_generator_translation_error']
            localization_generator_rotation_error = self.run_parameters['localization_generator_rotation_error']
            localization_generator_normalized_relative_translation_error = self.run_parameters['localization_generator_normalized_relative_translation_error']
            localization_generator_normalized_relative_rotation_error = self.run_parameters['localization_generator_normalized_relative_rotation_error']
        elif self.localization_node == 'ground_truth': 
            pass
        
        else:
            raise ValueError()

        hunter2_wheelbase = self.benchmark_configuration['hunter2_wheelbase']
        hunter2_min_turning_radius = float(hunter2_wheelbase/np.tan(max_steering_rad)) if 'max_steering_angle_deg' in self.run_parameters else None
        hunter2_footprint = self.benchmark_configuration['hunter2_footprint']
        hunter2_footprint_string = str(hunter2_footprint)

        turtlebot_wheelbase = self.benchmark_configuration['hunter2_wheelbase']
        turtlebot_min_turning_radius = float(hunter2_wheelbase / np.tan(max_steering_rad)) if 'max_steering_angle_deg' in self.run_parameters else None
        turtlebot_footprint = self.benchmark_configuration['turtlebot_footprint']
        turtlebot_footprint_string = str(turtlebot_footprint)

        #max_circumscribing_circle_radius = max(float(np.max(np.fabs(np.array(turtlebot_footprint)))),
        #                                       float(np.max(np.fabs(np.array(hunter2_footprint)))))  # largest radius for any robot's circumscribing circle
        max_circumscribing_circle_radius = float(np.max(np.fabs(np.array(turtlebot_footprint))))        # approx. 0.2
                                               
        #goal_obstacle_min_distance = 0.2 + max_circumscribing_circle_radius  # minimum distance between goals and obstacles, as the robots largest radius plus a margin TODO this stuff should be a run parameter
        goal_obstacle_min_distance = 0.3  # minimum distance between goals and obstacles
        # in case this value is changed, recompute all waypoints for each map. At the moment there is a standalone script which creates voronoi_graphs
        # and uses it to generate waypoints with node coordinates. Being the script a standalone and independent from the simulation, we need to pass this values as parameter of a launch file.
        # Hence, when changing this value, use voronoi.lanuch for each map passing the new goal_obstacle_min_distance as value.

        if robot_model == 'turtlebot3_waffle_performance_modelling':
            robot_drive_plugin_type = 'diff_drive_plugin'
        elif robot_model == 'hunter2':
            robot_drive_plugin_type = 'ackermann_drive_plugin'
        else:
            raise ValueError()
            
        # run variables
        self.aborted = False

        # prepare folder structure
        run_configuration_path = path.join(self.run_output_folder, "components_configuration")
        run_info_file_path = path.join(self.run_output_folder, "run_info.yaml")
        self.run_completed_file_path = path.join(self.run_output_folder, "RUN_COMPLETED")
        backup_file_if_exists(self.run_output_folder)
        os.mkdir(self.run_output_folder)
        os.mkdir(run_configuration_path)
        self.recorder_output_path = path.join(self.run_output_folder, "benchmark_data.bag")
        self.ros_log_directory = path.join(self.run_output_folder, "logs")
        os.mkdir(self.ros_log_directory)

        # components original configuration paths
        components_configurations_folder = path.expanduser(self.benchmark_configuration['components_configurations_folder'])
        original_supervisor_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['supervisor'])
        original_localization_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][self.localization_node])
        original_navigation_stack_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['navigation_stack'])
        self.original_rviz_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['rviz'])
        original_local_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][local_planner_node])
        original_global_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][global_planner_node])
        original_gazebo_world_model_path = path.join(self.environment_folder, "gazebo", "gazebo_environment.model")
        original_gazebo_robot_model_config_path = path.join(robots_dataset_folder, robot_model, "model.config")
        original_gazebo_robot_model_sdf_path = path.join(robots_dataset_folder, robot_model, "model.sdf")
        original_robot_urdf_path = path.join(robots_dataset_folder, robot_model, "robot.urdf")
        original_pedsim_config_path = self.scene_file_path

        # components configuration relative paths
        supervisor_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['supervisor'])
        localization_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][self.localization_node])
        navigation_stack_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['navigation_stack'])
        local_planner_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][local_planner_node])
        global_planner_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][global_planner_node])
        gazebo_world_model_relative_path = path.join("components_configuration", "gazebo", "gazebo_environment.model")
        gazebo_robot_model_config_relative_path = path.join("components_configuration", "gazebo", "robot", "model.config")
        gazebo_robot_model_sdf_relative_path = path.join("components_configuration", "gazebo", "robot", "model.sdf")
        robot_realistic_urdf_relative_path = path.join("components_configuration", "gazebo", "robot", "robot_realistic.urdf")
        pedsim_config_relative_path = path.join("components_configuration", "gazebo", "scene.xml")

        # components configuration paths in run folder
        self.supervisor_configuration_path = path.join(self.run_output_folder, supervisor_configuration_relative_path)
        self.localization_configuration_path = path.join(self.run_output_folder, localization_configuration_relative_path)
        self.navigation_stack_configuration_path = path.join(self.run_output_folder, navigation_stack_configuration_relative_path)
        self.local_planner_configuration_path = path.join(self.run_output_folder, local_planner_configuration_relative_path)
        self.global_planner_configuration_path = path.join(self.run_output_folder, global_planner_configuration_relative_path)
        self.gazebo_world_model_path = path.join(self.run_output_folder, gazebo_world_model_relative_path)
        gazebo_robot_model_config_path = path.join(self.run_output_folder, gazebo_robot_model_config_relative_path)
        gazebo_robot_model_sdf_path = path.join(self.run_output_folder, gazebo_robot_model_sdf_relative_path)
        self.robot_realistic_urdf_path = path.join(self.run_output_folder, robot_realistic_urdf_relative_path)
        self.pedsim_config_path = path.join(self.run_output_folder, pedsim_config_relative_path)

        # copy the configuration of the supervisor to the run folder and update its parameters
        with open(original_supervisor_configuration_path) as supervisor_configuration_file:
            supervisor_configuration = yaml.safe_load(supervisor_configuration_file)
        supervisor_configuration['run_index'] = self.run_index
        supervisor_configuration['run_output_folder'] = self.run_output_folder
        supervisor_configuration['pid_father'] = os.getpid()
        supervisor_configuration['use_sim_time'] = self.use_sim_time
        supervisor_configuration['ground_truth_map_info_path'] = self.map_info_file_path
        supervisor_configuration['goal_obstacle_min_distance'] = goal_obstacle_min_distance
        supervisor_configuration['goal_publication_type'] = 'action' if local_planner_node in ['dwa', 'teb', 'rpp'] else 'topic' 
        if self.localization_node == 'amcl':
            supervisor_configuration['estimated_pose_correction_topic'] = "/amcl_pose"
        elif self.localization_node == 'localization_generator':
            supervisor_configuration['estimated_pose_correction_topic'] = "/generated_pose"
        else:
            raise ValueError()

        if not path.exists(path.dirname(self.supervisor_configuration_path)):
            os.makedirs(path.dirname(self.supervisor_configuration_path))
        with open(self.supervisor_configuration_path, 'w') as supervisor_configuration_file:
            yaml.dump(supervisor_configuration, supervisor_configuration_file, default_flow_style=False)

        # copy the configuration of the localization to the run folder and update its parameters
        with open(original_localization_configuration_path) as localization_configuration_file:
            localization_configuration = yaml.safe_load(localization_configuration_file)
        if self.localization_node == 'amcl':
            localization_configuration['alpha1'] = amcl_alpha_1
            localization_configuration['alpha2'] = amcl_alpha_2
            localization_configuration['alpha3'] = amcl_alpha_3
            localization_configuration['alpha4'] = amcl_alpha_4
        elif self.localization_node == 'localization_generator':
            localization_configuration['update_pose_rate'] = localization_generator_update_rate
            localization_configuration['translation_error'] = localization_generator_translation_error
            localization_configuration['rotation_error'] = localization_generator_rotation_error
            localization_configuration['normalized_relative_translation_error'] = localization_generator_normalized_relative_translation_error
            localization_configuration['normalized_relative_rotation_error'] = localization_generator_normalized_relative_rotation_error
        elif self.localization_node == 'ground_truth': 
            pass
        
        else:
            raise ValueError()
        if not path.exists(path.dirname(self.localization_configuration_path)):
            os.makedirs(path.dirname(self.localization_configuration_path))
        with open(self.localization_configuration_path, 'w') as localization_configuration_file:
            yaml.dump(localization_configuration, localization_configuration_file, default_flow_style=False)

        # copy the configuration of the navigation_stack to the run folder and update its parameters
        with open(original_navigation_stack_configuration_path) as navigation_stack_configuration_file:
            navigation_stack_configuration = yaml.safe_load(navigation_stack_configuration_file)
        # navigation_stack_configuration['map_server']['ros__parameters']['yaml_filename'] = self.map_info_file_path
        if robot_model == 'turtlebot3_waffle_performance_modelling':
            navigation_stack_configuration['local_costmap']['footprint'] = turtlebot_footprint_string
            navigation_stack_configuration['global_costmap']['footprint'] = turtlebot_footprint_string
            #print(navigation_stack_configuration['local_costmap']['footprint'])
            #print(navigation_stack_configuration_file)
        elif robot_model == 'hunter2':
            navigation_stack_configuration['local_costmap']['footprint'] = hunter2_footprint_string
            navigation_stack_configuration['global_costmap']['footprint'] = hunter2_footprint_string
        else:
            raise ValueError()
        if not path.exists(path.dirname(self.navigation_stack_configuration_path)):
            os.makedirs(path.dirname(self.navigation_stack_configuration_path))
        with open(self.navigation_stack_configuration_path, 'w') as navigation_stack_configuration_file:
            yaml.dump(navigation_stack_configuration, navigation_stack_configuration_file, default_flow_style=False)

        # copy the configuration of global_planner to the run folder and update its parameters
        with open(original_global_planner_configuration_path) as global_planner_configuration_file:
            global_planner_configuration = yaml.safe_load(global_planner_configuration_file)
        # if global_planner_node == 'smac':
        #     if robot_model == 'turtlebot3_waffle_performance_modelling':
        #         global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = turtlebot_min_turning_radius
        #     elif robot_model == 'hunter2':
        #         global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = hunter2_min_turning_radius
        #     else:
        #         raise ValueError()
        if not path.exists(path.dirname(self.global_planner_configuration_path)):
            os.makedirs(path.dirname(self.global_planner_configuration_path))
        with open(self.global_planner_configuration_path, 'w') as global_planner_configuration_file:
            yaml.dump(global_planner_configuration, global_planner_configuration_file, default_flow_style=False)

        # copy the configuration of local_planner to the run folder and update its parameters
        with open(original_local_planner_configuration_path) as local_planner_configuration_file:
            local_planner_configuration = yaml.safe_load(local_planner_configuration_file)
        # if local_planner_node == 'teb':  # TODO
        #     if robot_model == 'turtlebot3_waffle_performance_modelling':
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['cmd_angle_instead_rotvel'] = False
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.type'] = "polygon"
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.vertices'] = turtlebot_footprint_string
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['wheelbase'] = turtlebot_wheelbase
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['min_turning_radius'] = turtlebot_min_turning_radius
        #     elif robot_model == 'hunter2':
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['cmd_angle_instead_rotvel'] = True
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.type'] = "polygon"
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.vertices'] = hunter2_footprint_string
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['wheelbase'] = hunter2_wheelbase
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['min_turning_radius'] = hunter2_min_turning_radius
        #     else:
        #         raise ValueError()
        # elif local_planner_node == 'rpp':
        #     if robot_model == 'turtlebot3_waffle_performance_modelling':
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['use_rotate_to_heading'] = True
        #     elif robot_model == 'hunter2':
        #         local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['use_rotate_to_heading'] = False
        #     else:
        #         raise ValueError()
        # elif local_planner_node == 'dwa':
        #     pass
        # else:
        #     raise ValueError()
        if not path.exists(path.dirname(self.local_planner_configuration_path)):
            os.makedirs(path.dirname(self.local_planner_configuration_path))
        with open(self.local_planner_configuration_path, 'w') as local_planner_configuration_file:
            yaml.dump(local_planner_configuration, local_planner_configuration_file, default_flow_style=False)

        # copy the configuration of the gazebo world model to the run folder and update its parameters
        gazebo_original_world_model_tree = et.parse(original_gazebo_world_model_path)
        gazebo_original_world_model_root = gazebo_original_world_model_tree.getroot()
        gazebo_original_world_model_root.findall(".//include[@include_id='robot_model']/uri")[0].text = path.join("model://", path.dirname(gazebo_robot_model_sdf_relative_path))
        if not path.exists(path.dirname(self.gazebo_world_model_path)):
            os.makedirs(path.dirname(self.gazebo_world_model_path))
        gazebo_original_world_model_tree.write(self.gazebo_world_model_path)

        # copy the configuration of the gazebo robot sdf model to the run folder and update its parameters
        gazebo_robot_model_sdf_tree = et.parse(original_gazebo_robot_model_sdf_path)
        gazebo_robot_model_sdf_root = gazebo_robot_model_sdf_tree.getroot()

        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/plugin[@name='laserscan_realistic_plugin']/frameName")[0].text = "base_scan"
        if alpha_1 == 0 and alpha_2 == 0 and alpha_3 == 0 and alpha_4 == 0:
            gazebo_robot_model_sdf_root.findall(".//plugin[@name='{robot_drive_plugin_type}']/odometrySource".format(robot_drive_plugin_type=robot_drive_plugin_type))[0].text = "world"
        else:
            gazebo_robot_model_sdf_root.findall(".//plugin[@name='{robot_drive_plugin_type}']/odometrySource".format(robot_drive_plugin_type=robot_drive_plugin_type))[0].text = "parametric_error_model"  # TODO string instead of int?
            gazebo_robot_model_sdf_root.findall(".//plugin[@name='{robot_drive_plugin_type}']/alpha1".format(robot_drive_plugin_type=robot_drive_plugin_type))[0].text = str(alpha_1)
            gazebo_robot_model_sdf_root.findall(".//plugin[@name='{robot_drive_plugin_type}']/alpha2".format(robot_drive_plugin_type=robot_drive_plugin_type))[0].text = str(alpha_2)
            gazebo_robot_model_sdf_root.findall(".//plugin[@name='{robot_drive_plugin_type}']/alpha3".format(robot_drive_plugin_type=robot_drive_plugin_type))[0].text = str(alpha_3)
            gazebo_robot_model_sdf_root.findall(".//plugin[@name='{robot_drive_plugin_type}']/alpha4".format(robot_drive_plugin_type=robot_drive_plugin_type))[0].text = str(alpha_4)

        if robot_model == 'hunter2':
            gazebo_robot_model_sdf_root.findall(".//plugin[@name='{robot_drive_plugin_type}']/maxSteer".format(robot_drive_plugin_type=robot_drive_plugin_type))[0].text = str(max_steering_rad*1.1)
        if not path.exists(path.dirname(gazebo_robot_model_sdf_path)):
            os.makedirs(path.dirname(gazebo_robot_model_sdf_path))
        gazebo_robot_model_sdf_tree.write(gazebo_robot_model_sdf_path)

        # copy the configuration of the gazebo robot model to the run folder
        if not path.exists(path.dirname(gazebo_robot_model_config_path)):
            os.makedirs(path.dirname(gazebo_robot_model_config_path))
        shutil.copyfile(original_gazebo_robot_model_config_path, gazebo_robot_model_config_path)

        # copy the configuration of the robot urdf to the run folder and update the link names for realistic data
        robot_realistic_urdf_tree = et.parse(original_robot_urdf_path)
        if not path.exists(path.dirname(self.robot_realistic_urdf_path)):
            os.makedirs(path.dirname(self.robot_realistic_urdf_path))
        robot_realistic_urdf_tree.write(self.robot_realistic_urdf_path)

        # copy the configuration of the scene.xml file to the run folder
        gazebo_original_pedsim_tree = et.parse(original_pedsim_config_path)
        gazebo_original_pedsim_root = gazebo_original_pedsim_tree.getroot()
        if not path.exists(path.dirname(self.pedsim_config_path)):
            os.makedirs(path.dirname(self.pedsim_config_path))

        # compute voronoi_graph in order to generate waypoints
        self.ground_truth_map = ground_truth_map.GroundTruthMap(self.map_info_file_path)
        voronoi_graph = self.ground_truth_map.deleaved_reduced_voronoi_graph(minimum_radius=goal_obstacle_min_distance).copy()
        
        # parse scene.xml
        # gazebo_original_pedsim_tree = et.parse(self.scene_file_path)
        # gazebo_original_pedsim_root = gazebo_original_pedsim_tree.getroot()

        # remove waypoints already present 
        # for child in gazebo_original_pedsim_root.findall('waypoint'):
        #     gazebo_original_pedsim_root.remove(child)
        
        # add waypoints from voronoi graph
        for i in voronoi_graph.nodes:
            x = voronoi_graph.nodes[i]['vertex'][0]
            y = voronoi_graph.nodes[i]['vertex'][1]
            new_waypoint = Element('waypoint', attrib={'id': 'waypoint_id_' + str(i), 'x': str(x), 'y': str(y), 'r': str(1)})
            gazebo_original_pedsim_root.append(new_waypoint)

        # get the robot position from gazebo_environment.model (starting position is different according to each environment)
        pose_string = gazebo_original_world_model_root.findall(".//include[@include_id='robot_model']/pose")[0].text
        pose_string = pose_string.split(' ')
        robot_pose_x = float(pose_string[0])
        robot_pose_y = float(pose_string[1])
        robot_pose_z = float(pose_string[2])
        print("Robot pose -> x:" + str(robot_pose_x), " y:" + str(robot_pose_y), " z:" + str(robot_pose_z))

        # find the goal 
        
        # in case the graph has multiple unconnected components, remove the components with less than two nodes
        too_small_voronoi_graph_components = list(filter(lambda component: len(component) < 2, nx.connected_components(voronoi_graph)))

        for graph_component in too_small_voronoi_graph_components:
            voronoi_graph.remove_nodes_from(graph_component)

        # select the node pseudo-randomly using the run number
        # the list of indices is always shuffled the same way (seed = 0), so each run number will always correspond to the same Voronoi node
        nil = copy.copy(list(voronoi_graph.nodes))  # list of the indices of the nodes in voronoi_graph.nodes
        random.Random(0).shuffle(nil)
        self.pseudo_random_voronoi_index = nil[self.run_index % len(nil)]

        # convert Voronoi node to pose
        self.goal_pose = PoseStamped()
        self.goal_pose.pose = Pose()
        self.goal_pose.pose.position.x, self.goal_pose.pose.position.y = voronoi_graph.nodes[self.pseudo_random_voronoi_index]['vertex']
        print("Goal x: " + str(self.goal_pose.pose.position.x), "y: " + str(self.goal_pose.pose.position.y))

        # given starting robot position and goal position, find the shortest path from goal to start robot pos

        # first find the node id corresponding to the goal position
        node_id = None
        for i in voronoi_graph.nodes:
            x = voronoi_graph.nodes[i]['vertex'][0]
            y = voronoi_graph.nodes[i]['vertex'][1]
            if (x == self.goal_pose.pose.position.x and y == self.goal_pose.pose.position.y):
                node_id = i
                x_goal = x
                y_goal = y
        # then compute id of the nearest node to robot position (do it manually for the moment) TODO
        start_id = 1191

        # compute shortest path from node_id to start_id
        print("Compute shortest path from", node_id, "to", start_id)
        shortest_path = nx.dijkstra_path(voronoi_graph, node_id, start_id)
        print(shortest_path)

        # # remove all old agents 
        # for child in gazebo_original_pedsim_root.findall('agent'):
        #     gazebo_original_pedsim_root.remove(child)
        
        # prepare data for the new agent of type 2 to add in the xml (necessary so that pedestrians avoid the robot) and add it
        x_agent = 0.0   #these are the default values for agent of type 2 according to pedsim
        y_agent = 0.0
        dx = 0.5
        dy = 0.5
        n = 1
        type = 2
        new_agent = Element('agent', attrib={'x': str(x_agent), 
                                             'y': str(y_agent), 
                                             'dx': str(dx), 
                                             'dy': str(dy), 
                                             'n': str(n), 
                                             'type': str(type)})
        gazebo_original_pedsim_root.append(new_agent) 
        
        # now prepare the agents which will follow the shortest path
        x_agent = x_goal
        y_agent = y_goal
        print(x_goal, y_goal)
        dx = 0.5
        dy = 0.5
        n = pedestrian_number
        type = 10
        new_agent = Element('agent', attrib={'x': str(x_agent), 
                                             'y': str(y_agent), 
                                             'dx': str(dx), 
                                             'dy': str(dy), 
                                             'n': str(n), 
                                             'type': str(type)})

        # add waypoints found in the shortest path as sub-elements of the new agent
        for i in shortest_path:
            et.SubElement(new_agent, 'addwaypoint', attrib = {'id': 'waypoint_id_' + str(i)})

        # for i in reversed(shortest_path):
        #     #print(i)
        #     et.SubElement(new_agent, 'addwaypoint', attrib = {'id': 'waypoint_id_' + str(i)})
        
        gazebo_original_pedsim_root.append(new_agent) 


        # write to the file in the run folder  
        gazebo_original_pedsim_tree.write(self.pedsim_config_path)

        # write run info to file
        run_info_dict = dict()
        run_info_dict["run_id"] = self.run_id
        run_info_dict["run_folder"] = self.run_output_folder
        run_info_dict["environment_folder"] = self.environment_folder
        run_info_dict["run_parameters"] = self.run_parameters
        run_info_dict["local_components_configuration"] = {
            'supervisor': supervisor_configuration_relative_path,
            'localization': localization_configuration_relative_path,
            'navigation_stack': navigation_stack_configuration_relative_path,
            'local_planner_configuration_relative_path': local_planner_configuration_relative_path,
            'global_planner_configuration_relative_path': global_planner_configuration_relative_path,
            'gazebo_world_model': gazebo_world_model_relative_path,
            'gazebo_robot_model_sdf': gazebo_robot_model_sdf_relative_path,
            'gazebo_robot_model_config': gazebo_robot_model_config_relative_path,
            'robot_realistic_urdf': robot_realistic_urdf_relative_path,
            'pedsim_config': pedsim_config_relative_path,
        }

        with open(run_info_file_path, 'w') as run_info_file:
            yaml.dump(run_info_dict, run_info_file, default_flow_style=False)

        # log packages and software versions and status
        log_packages_and_repos(source_workspace_path=self.benchmark_configuration['source_workspace_path'], log_dir_path=path.join(self.run_output_folder, "software_versions_log"), use_rospack=False)

    def log(self, event):

        if not path.exists(self.benchmark_log_path):
            with open(self.benchmark_log_path, 'a') as output_file:
                output_file.write("timestamp, run_id, event\n")

        t = time.time()

        print_info("t: {t}, run: {run_id}, event: {event}".format(t=t, run_id=self.run_id, event=event))
        try:
            with open(self.benchmark_log_path, 'a') as output_file:
                output_file.write("{t}, {run_id}, {event}\n".format(t=t, run_id=self.run_id, event=event))
        except IOError as e:
            print_error("benchmark_log: could not write event to file: {t}, {run_id}, {event}".format(t=t, run_id=self.run_id, event=event))
            print_error(e)

    def execute_run(self):
        self.log(event="run_start")
        local_planner_node = self.run_parameters['local_planner_node']
        pedestrian_number = self.run_parameters['pedestrian_number']

        # declare components
        roscore = Component('roscore', 'local_planning_performance_modelling', 'roscore.launch')

        supervisor = Component('supervisor', 'local_planning_performance_modelling', 'local_planning_benchmark_supervisor.launch', {
            'configuration': self.supervisor_configuration_path,
            'log_path': self.ros_log_directory,
        })
        recorder = Component('recorder', 'local_planning_performance_modelling', 'recorder.launch', {
            'recorder_output_path': self.recorder_output_path,
            'log_path': self.ros_log_directory,
        })
        environment = Component('environment', 'local_planning_performance_modelling', 'environment.launch', {
            'urdf': self.robot_realistic_urdf_path,
            'world': self.gazebo_world_model_path,
            'launch_rviz': self.launch_rviz,
            'launch_gzclient': self.launch_gzclient,
            'gazebo_model_path_env_var': self.gazebo_model_path_env_var,
            'gazebo_plugin_path_env_var': self.gazebo_plugin_path_env_var,
            'params_file': self.navigation_stack_configuration_path,
            'rviz_config_file': self.original_rviz_configuration_path,
            'log_path': self.ros_log_directory,
        })

        if pedestrian_number > 0:
            pedsim = Component('pedsim', 'local_planning_performance_modelling', 'pedsim.launch', {
            'scene_file': self.pedsim_config_path,
            'pedestrian_number': self.run_parameters['pedestrian_number'],
            'pedestrian_max_vel':self.run_parameters['pedestrian_max_vel'],
            })

        if local_planner_node == 'arena':
            navigation_arena = Component('navigation_arena', 'local_planning_performance_modelling', 'navigation_arena.launch', {
                        'local_planner_params_file': self.local_planner_configuration_path,
                        'global_planner_params_file': self.global_planner_configuration_path,
                        'nav_params_file': self.navigation_stack_configuration_path,
                        'map': self.map_info_file_path,
                        'log_path': self.ros_log_directory,
            })
        else:
            navigation = Component('navigation', 'local_planning_performance_modelling', 'navigation.launch', {
                        'local_planner_params_file': self.local_planner_configuration_path,
                        'global_planner_params_file': self.global_planner_configuration_path,
                        'nav_params_file': self.navigation_stack_configuration_path,
                        'map': self.map_info_file_path,
                        'log_path': self.ros_log_directory,
            })
    
        #if self.localization_node == 'ground_truth':
        localization = Component('localization', 'local_planning_performance_modelling', '{localization_node}.launch'.format(localization_node=self.localization_node), {
            'localization_params_file': self.localization_configuration_path,
            'log_path': self.ros_log_directory,
        })

        # set gazebo's environment variables
        os.environ['GAZEBO_MODEL_PATH'] = self.gazebo_model_path_env_var
        # os.environ['GAZEBO_RESOURCE_PATH'] = self.gazebo_resource_path_env_var  # TODO still needed?
        os.environ['GAZEBO_PLUGIN_PATH'] = self.gazebo_plugin_path_env_var

        # launch roscore and setup a node to monitor ros
        roscore.launch()
        rospy.init_node("benchmark_monitor", anonymous=True)

        # launch components
        if not self.do_not_record:
            recorder.launch()
        if pedestrian_number > 0:
            pedsim.launch()
        environment.launch()
        localization.launch()
        if local_planner_node == 'arena':
            navigation_arena.launch()
        else: 
            navigation.launch()
        supervisor.launch()

        # launch components and wait for the supervisor to finish
        self.log(event="waiting_supervisor_finish")
        supervisor.wait_to_finish()
        self.log(event="supervisor_shutdown")

        # check if the rosnode is still ok, otherwise the ros infrastructure has been shutdown and the benchmark is aborted
        if rospy.is_shutdown():
            print_error("execute_run: supervisor finished by ros_shutdown")
            self.aborted = True

        # shut down components
        supervisor.shutdown()
        if local_planner_node == 'arena':
            navigation_arena.shutdown()
        else: 
            navigation.shutdown()
        localization.shutdown()
        if pedestrian_number > 0:
            pedsim.shutdown()
        environment.shutdown()
        if not self.do_not_record:
            recorder.shutdown()
        roscore.shutdown()
        self.log(event="components_shutdown")

        with open(self.run_completed_file_path, 'w') as _:
            pass

        self.log(event="run_end")

