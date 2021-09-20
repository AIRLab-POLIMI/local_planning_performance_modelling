#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import shutil
import traceback

import yaml


import xml.etree.ElementTree as et
from xml.etree.ElementTree import tostring


import time
from os import path
import numpy as np

from performance_modelling_py.utils import backup_file_if_exists, print_info, print_error
from performance_modelling_py.component_proxies.ros2_component import Component, ComponentsLauncher


class BenchmarkRun(object):
    def __init__(self, run_id, run_output_folder, benchmark_log_path, environment_folder, parameters_combination_dict, benchmark_configuration_dict, show_ros_info, headless):

        # run configuration
        self.run_id = run_id
        self.run_output_folder = run_output_folder
        self.benchmark_log_path = benchmark_log_path
        self.run_parameters = parameters_combination_dict
        self.benchmark_configuration = benchmark_configuration_dict
        self.components_ros_output = 'screen' if show_ros_info else 'log'
        self.headless = headless
        self.use_sim_time = True
        
        # environment parameters
        robots_dataset_folder = path.expanduser(self.benchmark_configuration['robots_dataset'])
        print_info('robots_dataset_folder', robots_dataset_folder)

        self.environment_folder = environment_folder
        self.map_info_file_path = path.join(environment_folder, "data", "map.yaml")
        self.gazebo_model_path_env_var = ":".join(map(
            lambda p: path.expanduser(p),
            self.benchmark_configuration['gazebo_model_path_env_var'] + [path.dirname(path.abspath(self.environment_folder)), self.run_output_folder]
        ))
        self.gazebo_plugin_path_env_var = ":".join(map(
            lambda p: path.expanduser(p),
            self.benchmark_configuration['gazebo_plugin_path_env_var']
        ))


        robot_model = self.run_parameters['robot_model']
        local_planner_node = self.run_parameters['local_planner_node']
        global_planner_node = self.run_parameters['global_planner_node']
        max_steering_angle_deg = self.run_parameters['max_steering_angle_deg'] if robot_model == 'hunter2' else None


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
        backup_file_if_exists(self.run_output_folder)
        os.mkdir(self.run_output_folder)
        os.mkdir(run_configuration_path)

        # components original configuration paths 
        components_configurations_folder = path.expanduser(self.benchmark_configuration['components_configurations_folder'])
        #gazebo_robot_ds = path.expanduser(self.benchmark_configuration['gazebo_robot_ds'])
        original_supervisor_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['supervisor'])
        #original_nav2_amcl_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['nav2_amcl'])
        original_nav2_navigation_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['nav2_navigation'])
        self.original_rviz_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['rviz'])
        original_local_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][local_planner_node])
        original_global_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][global_planner_node])
        original_gazebo_world_model_path = path.join(environment_folder, "gazebo", "gazebo_environment.model")
        original_gazebo_robot_model_config_path = path.join(robots_dataset_folder, robot_model, "model.config")
        original_gazebo_robot_model_sdf_path = path.join(robots_dataset_folder, robot_model, "model.sdf")
        original_robot_urdf_path = path.join(robots_dataset_folder, robot_model, "robot.urdf")

        # components configuration relative paths
        supervisor_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['supervisor'])
        #nav2_amcl_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['nav2_amcl'])
        nav2_navigation_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['nav2_navigation'])
        local_planner_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][local_planner_node])
        global_planner_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][global_planner_node])
        gazebo_world_model_relative_path = path.join("components_configuration", "gazebo", "gazebo_environment.model")
        gazebo_robot_model_config_relative_path = path.join("components_configuration", "gazebo", "robot", "model.config")
        gazebo_robot_model_sdf_relative_path = path.join("components_configuration", "gazebo", "robot", "model.sdf")
        #robot_gt_urdf_relative_path = path.join("components_configuration", "gazebo", "robot_gt.urdf")
        robot_realistic_urdf_relative_path = path.join("components_configuration", "gazebo", "robot", "robot_realistic.urdf")

        # components configuration paths in run folder
        self.supervisor_configuration_path = path.join(self.run_output_folder, supervisor_configuration_relative_path)
        #self.nav2_amcl_configuration_path = path.join(self.run_output_folder, nav2_amcl_configuration_relative_path)
        self.nav2_navigation_configuration_path = path.join(self.run_output_folder, nav2_navigation_configuration_relative_path)
        self.local_planner_configuration_path = path.join(self.run_output_folder, local_planner_configuration_relative_path)
        self.global_planner_configuration_path = path.join(self.run_output_folder, global_planner_configuration_relative_path)
        self.gazebo_world_model_path = path.join(self.run_output_folder, gazebo_world_model_relative_path)
        gazebo_robot_model_config_path = path.join(self.run_output_folder, gazebo_robot_model_config_relative_path)
        print_info('gazebo_robot_model_config_path', gazebo_robot_model_config_path)
        gazebo_robot_model_sdf_path = path.join(self.run_output_folder, gazebo_robot_model_sdf_relative_path)
        print_info('gazebo_robot_model_sdf_path', gazebo_robot_model_sdf_path)
        #self.robot_gt_urdf_path = path.join(self.run_output_folder, robot_gt_urdf_relative_path)
        self.robot_realistic_urdf_path = path.join(self.run_output_folder, robot_realistic_urdf_relative_path)
        print_info('robot_realistic_urdf_path',  self.robot_realistic_urdf_path)

        # copy the configuration of the supervisor to the run folder and update its parameters
        with open(original_supervisor_configuration_path) as supervisor_configuration_file:
            supervisor_configuration = yaml.safe_load(supervisor_configuration_file)
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['run_output_folder'] = self.run_output_folder
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['pid_father'] = os.getpid()
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['use_sim_time'] = self.use_sim_time
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['ground_truth_map_info_path'] = self.map_info_file_path
        if not path.exists(path.dirname(self.supervisor_configuration_path)):
            os.makedirs(path.dirname(self.supervisor_configuration_path))
        with open(self.supervisor_configuration_path, 'w') as supervisor_configuration_file:
            yaml.dump(supervisor_configuration, supervisor_configuration_file, default_flow_style=False)
            
        # copy the configuration of the nav2_navigation to the run folder and update its parameters
        with open(original_nav2_navigation_configuration_path) as nav2_navigation_configuration_file:
            nav2_navigation_configuration = yaml.safe_load(nav2_navigation_configuration_file)
        nav2_navigation_configuration['map_server']['ros__parameters']['yaml_filename'] = self.map_info_file_path
        if not path.exists(path.dirname(self.nav2_navigation_configuration_path)):
            os.makedirs(path.dirname(self.nav2_navigation_configuration_path))
        with open(self.nav2_navigation_configuration_path, 'w') as nav2_navigation_configuration_file:
            yaml.dump(nav2_navigation_configuration, nav2_navigation_configuration_file, default_flow_style=False)

        # copy the configuration of global_planner to the run folder
        # if not path.exists(path.dirname(self.global_planner_configuration_path)):
        #     os.makedirs(path.dirname(self.global_planner_configuration_path))
        # shutil.copyfile(original_global_planner_configuration_path, self.global_planner_configuration_path)

        # copy the configuration of local_planner to the run folder
        # with open(original_local_planner_configuration_path) as local_planner_configuration_file:
        #     local_planner_configuration = yaml.safe_load(local_planner_configuration_file)
        # if local_planner_node == 'teb' and robot_model == 'hunter2':
        #     local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['min_turning_radius'] = 1.0
        # if not path.exists(path.dirname(self.local_planner_configuration_path)):
        #     os.makedirs(path.dirname(self.local_planner_configuration_path))
        # with open(self.local_planner_configuration_path, 'w') as local_planner_configuration_file:
        #     yaml.dump(local_planner_configuration, local_planner_configuration_file, default_flow_style=False)

        # copy the configuration of global_planner to the run folder
        with open(original_global_planner_configuration_path) as global_planner_configuration_file:
            global_planner_configuration = yaml.safe_load(global_planner_configuration_file)
        if local_planner_node == 'teb' and robot_model == 'hunter2' and max_steering_angle_deg == '50':
            global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = 0.55
        elif local_planner_node == 'teb' and robot_model == 'hunter2' and max_steering_angle_deg == '22':
            global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = 1.61
        else:
            global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = 2.43
        if not path.exists(path.dirname(self.global_planner_configuration_path)):
            os.makedirs(path.dirname(self.global_planner_configuration_path))
        with open(self.global_planner_configuration_path, 'w') as global_planner_configuration_file:
            yaml.dump(global_planner_configuration, global_planner_configuration_file, default_flow_style=False)

        # copy the configuration of local_planner to the run folder
        if not path.exists(path.dirname(self.local_planner_configuration_path)):
            os.makedirs(path.dirname(self.local_planner_configuration_path))
        shutil.copyfile(original_local_planner_configuration_path, self.local_planner_configuration_path)
        

        # copy the configuration of the gazebo world model to the run folder and update its parameters
        gazebo_original_world_model_tree = et.parse(original_gazebo_world_model_path)
        gazebo_original_world_model_root = gazebo_original_world_model_tree.getroot()

        #print_info('lol',path.dirname(gazebo_robot_model_sdf_relative_path))

        gazebo_original_world_model_root.findall(".//include[@include_id='robot_model']/uri")[0].text = path.join("model://", path.dirname(gazebo_robot_model_sdf_relative_path))
        if not path.exists(path.dirname(self.gazebo_world_model_path)):
            os.makedirs(path.dirname(self.gazebo_world_model_path))
        gazebo_original_world_model_tree.write(self.gazebo_world_model_path)

        # copy the configuration of the gazebo robot sdf model to the run folder and update its parameters
        gazebo_robot_model_sdf_tree = et.parse(original_gazebo_robot_model_sdf_path)
        gazebo_robot_model_sdf_root = gazebo_robot_model_sdf_tree.getroot()
        
        #rough_string = tostring(gazebo_robot_model_sdf_root, 'utf-8', method="xml")
        #print('asdfghjklç', rough_string)
        
        if robot_model == 'hunter2':
            max_steering_rad = (max_steering_angle_deg/180.0) * np.pi

        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/plugin[@name='laserscan_realistic_plugin']/frame_name")[0].text = "base_scan"
        gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/odometry_source")[0].text = "1"
        if robot_model == 'hunter2':
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/max_steer")[0].text = str(max_steering_rad)
        if not path.exists(path.dirname(gazebo_robot_model_sdf_path)):
            os.makedirs(path.dirname(gazebo_robot_model_sdf_path))
        gazebo_robot_model_sdf_tree.write(gazebo_robot_model_sdf_path)

        # copy the configuration of the gazebo robot model to the run folder
        if not path.exists(path.dirname(gazebo_robot_model_config_path)):
            os.makedirs(path.dirname(gazebo_robot_model_config_path))
        shutil.copyfile(original_gazebo_robot_model_config_path, gazebo_robot_model_config_path)

        # copy the configuration of the robot urdf to the run folder and update the link names for ground truth data
        #robot_gt_urdf_tree = et.parse(original_robot_urdf_path)
        #robot_gt_urdf_root = robot_gt_urdf_tree.getroot()
        #for link_element in robot_gt_urdf_root.findall(".//link"):
        #     if link_element.attrib['name'] == "base_link":  # currently (Eloquent) base_link is hardcoded in the navigation stack so it cannot be renamed
        #         continue
        #     link_el #print(navigation_params)ement.attrib['name'] = "{}_gt".format(link_element.attrib['name'])
        # for joint_link_element in robot_gt_urdf_root.findall(".//*[@link]"):
        #     if joint_link_element.attrib['link'] == "base_link":  # currently (Eloquent) base_link is hardcoded in the navigation stack so it cannot be renamed
        #         continue
        #     joint_link_element.attrib['link'] = "{}_gt".format(joint_link_element.attrib['link'])
        # if not path.exists(path.dirname(self.robot_gt_urdf_path)):
        #     os.makedirs(path.dirname(self.robot_gt_urdf_path))
        # robot_gt_urdf_tree.write(self.robot_gt_urdf_path)

        # copy the configuration of the robot urdf to the run folder and update the link names for realistic data
        robot_realistic_urdf_tree = et.parse(original_robot_urdf_path)
        robot_realistic_urdf_root = robot_realistic_urdf_tree.getroot()
        # for link_element in robot_realistic_urdf_root.findall(".//link"):
        #     link_element.attrib['name'] = "{}_realistic".format(link_element.attrib['name'])
        # for joint_link_element in robot_realistic_urdf_root.findall(".//*[@link]"):
        #     joint_link_element.attrib['link'] = "{}_realistic".format(joint_link_element.attrib['link'])
        if not path.exists(path.dirname(self.robot_realistic_urdf_path)):
            os.makedirs(path.dirname(self.robot_realistic_urdf_path))
        robot_realistic_urdf_tree.write(self.robot_realistic_urdf_path)

        # write run info to file
        run_info_dict = dict()
        run_info_dict["run_id"] = self.run_id
        run_info_dict["run_folder"] = self.run_output_folder
        run_info_dict["environment_folder"] = environment_folder
        run_info_dict["run_parameters"] = self.run_parameters
        run_info_dict["local_components_configuration"] = {
            'supervisor': supervisor_configuration_relative_path,
            #'nav2_amcl': nav2_amcl_configuration_relative_path,
            'nav2_navigation': nav2_navigation_configuration_relative_path,
            'gazebo_world_model': gazebo_world_model_relative_path,
            'gazebo_robot_model_sdf': gazebo_robot_model_sdf_relative_path,
            'gazebo_robot_model_config': gazebo_robot_model_config_relative_path,
            #'robot_gt_urdf': robot_gt_urdf_relative_path,
            'robot_realistic_urdf': robot_realistic_urdf_relative_path,
        }

        with open(run_info_file_path, 'w') as run_info_file:
            yaml.dump(run_info_dict, run_info_file, default_flow_style=False)

    def log(self, event):

        if not path.exists(self.benchmark_log_path):
            with open(self.benchmark_log_path, 'a') as output_file:
                output_file.write("timestamp, run_id, event\n")

        t = time.time()

        print_info(f"t: {t}, run: {self.run_id}, event: {event}")
        try:
            with open(self.benchmark_log_path, 'a') as output_file:
                output_file.write(f"{t}, {self.run_id}, {event}\n")
        except IOError as e:
            print_error(f"benchmark_log: could not write event to file: {t}, {self.run_id}, {event}")
            print_error(e)


    def execute_run(self):

        
        print_info(self.gazebo_model_path_env_var)

        supervisor_params = {
            'configuration': self.supervisor_configuration_path,
            'use_sim_time': self.use_sim_time
        }

        environment_params = {
            'urdf': self.robot_realistic_urdf_path,
            'world': self.gazebo_world_model_path,
            'headless': True,
            'gazebo_model_path_env_var': self.gazebo_model_path_env_var,
            'gazebo_plugin_path_env_var': self.gazebo_plugin_path_env_var,
            'params_file': self.nav2_navigation_configuration_path,
            'rviz_config_file': self.original_rviz_configuration_path
        }

        navigation_params = {
            'urdf': self.robot_realistic_urdf_path,
            'world': self.gazebo_world_model_path,
            'headless': True,
            'gazebo_model_path_env_var': self.gazebo_model_path_env_var,
            'gazebo_plugin_path_env_var': self.gazebo_plugin_path_env_var,
            'params_file': self.nav2_navigation_configuration_path,
            'rviz_config_file': self.original_rviz_configuration_path
        }

        local_planner_params = {
            'params_file': self.local_planner_configuration_path
        }

        global_planner_params = {
            'params_file': self.global_planner_configuration_path
        }

        # declare components
        supervisor = Component('supervisor', 'local_planning_performance_modelling', 'local_planning_benchmark_supervisor.launch.py', supervisor_params)
        environment = Component('launch','local_planning_performance_modelling', 'environment.launch.py', environment_params)
        navigation = Component('launch','local_planning_performance_modelling', 'navigation.launch.py', navigation_params)
        local_planner = Component('launch','local_planning_performance_modelling', 'local_planner.launch.py', local_planner_params)
        global_planner = Component('launch','local_planning_performance_modelling', 'global_planner.launch.py', global_planner_params)


        # add components to launcher
        components_launcher = ComponentsLauncher()
        # if not self.headless:
        #     components_launcher.add_component(rviz)
        # components_launcher.add_component(rviz)
        # recorder.launch()
        components_launcher.add_component(supervisor)
        components_launcher.add_component(environment)
        components_launcher.add_component(navigation)
        components_launcher.add_component(local_planner)
        components_launcher.add_component(global_planner)

        #print_info(launch_params)
    
        # launch components and wait for the supervisor to finish
        self.log(event="waiting_supervisor_finish")
        components_launcher.launch()
        self.log(event="supervisor_shutdown")

        # make sure remaining components have shutdown
        components_launcher.shutdown()
        print_info("execute_run: components shutdown completed")

        # compute all relevant metrics and visualisations
        # noinspection PyBroadException
#        try:
#            self.log(event="start_compute_metrics")
#            compute_metrics(self.run_output_folder)
#        except:
#            print_error("failed metrics computation")
#            print_error(traceback.format_exc())


        self.log(event="run_end")
        print_info(f"run {self.run_id} completed")

        
