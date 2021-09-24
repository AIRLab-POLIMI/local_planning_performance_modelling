#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import shutil

import yaml

import xml.etree.ElementTree as et

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

        self.run_index = self.run_parameters['run_index']
        robot_model = self.run_parameters['robot_model']
        local_planner_node = self.run_parameters['local_planner_node']
        global_planner_node = self.run_parameters['global_planner_node']
        max_steering_angle_deg = self.run_parameters['max_steering_angle_deg'] if 'max_steering_angle_deg' in self.run_parameters else None
        max_steering_rad = (max_steering_angle_deg/180.0) * np.pi if 'max_steering_angle_deg' in self.run_parameters else None
        wheelbase = self.benchmark_configuration['hunter2_wheelbase'] if 'max_steering_angle_deg' in self.run_parameters else None
        min_turning_radius = float(wheelbase/np.tan(max_steering_rad)) if 'max_steering_angle_deg' in self.run_parameters else None
        hunter2_footprint = self.benchmark_configuration['hunter2_footprint']
        hunter2_footprint_string = str(hunter2_footprint)
        turtlebot_radius = self.benchmark_configuration['turtlebot_radius']
        max_circumscribing_circle_radius = max(turtlebot_radius, float(np.max(np.fabs(np.array(hunter2_footprint)))))  # largest radius for any robot's circumscribing circle
        goal_obstacle_min_distance = 0.2 + max_circumscribing_circle_radius  # minimum distance between goals and obstacles, as the robots largest radius plus a margin TODO this stuff should be a run parameter

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
        original_supervisor_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['supervisor'])
        original_nav2_navigation_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['nav2_navigation'])
        original_behaviour_tree_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['behaviour_tree'])
        self.original_rviz_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['rviz'])
        original_local_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][local_planner_node])
        original_global_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][global_planner_node])
        original_gazebo_world_model_path = path.join(environment_folder, "gazebo", "gazebo_environment.model")
        original_gazebo_robot_model_config_path = path.join(robots_dataset_folder, robot_model, "model.config")
        original_gazebo_robot_model_sdf_path = path.join(robots_dataset_folder, robot_model, "model.sdf")
        original_robot_urdf_path = path.join(robots_dataset_folder, robot_model, "robot.urdf")

        # components configuration relative paths
        supervisor_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['supervisor'])
        nav2_navigation_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['nav2_navigation'])
        behaviour_tree_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['behaviour_tree'])
        local_planner_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][local_planner_node])
        global_planner_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][global_planner_node])
        gazebo_world_model_relative_path = path.join("components_configuration", "gazebo", "gazebo_environment.model")
        gazebo_robot_model_config_relative_path = path.join("components_configuration", "gazebo", "robot", "model.config")
        gazebo_robot_model_sdf_relative_path = path.join("components_configuration", "gazebo", "robot", "model.sdf")
        robot_realistic_urdf_relative_path = path.join("components_configuration", "gazebo", "robot", "robot_realistic.urdf")

        # components configuration paths in run folder
        self.supervisor_configuration_path = path.join(self.run_output_folder, supervisor_configuration_relative_path)
        self.nav2_navigation_configuration_path = path.join(self.run_output_folder, nav2_navigation_configuration_relative_path)
        behaviour_tree_configuration_path = path.join(self.run_output_folder, behaviour_tree_configuration_relative_path)
        self.local_planner_configuration_path = path.join(self.run_output_folder, local_planner_configuration_relative_path)
        self.global_planner_configuration_path = path.join(self.run_output_folder, global_planner_configuration_relative_path)
        self.gazebo_world_model_path = path.join(self.run_output_folder, gazebo_world_model_relative_path)
        gazebo_robot_model_config_path = path.join(self.run_output_folder, gazebo_robot_model_config_relative_path)
        gazebo_robot_model_sdf_path = path.join(self.run_output_folder, gazebo_robot_model_sdf_relative_path)
        self.robot_realistic_urdf_path = path.join(self.run_output_folder, robot_realistic_urdf_relative_path)

        # copy the configuration of the supervisor to the run folder and update its parameters
        with open(original_supervisor_configuration_path) as supervisor_configuration_file:
            supervisor_configuration = yaml.safe_load(supervisor_configuration_file)
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['run_index'] = self.run_index
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['run_output_folder'] = self.run_output_folder
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['pid_father'] = os.getpid()
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['use_sim_time'] = self.use_sim_time
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['ground_truth_map_info_path'] = self.map_info_file_path
        supervisor_configuration['local_planning_benchmark_supervisor']['ros__parameters']['goal_obstacle_min_distance'] = goal_obstacle_min_distance
        if not path.exists(path.dirname(self.supervisor_configuration_path)):
            os.makedirs(path.dirname(self.supervisor_configuration_path))
        with open(self.supervisor_configuration_path, 'w') as supervisor_configuration_file:
            yaml.dump(supervisor_configuration, supervisor_configuration_file, default_flow_style=False)

        # copy the configuration of the nav2_navigation to the run folder and update its parameters
        with open(original_nav2_navigation_configuration_path) as nav2_navigation_configuration_file:
            nav2_navigation_configuration = yaml.safe_load(nav2_navigation_configuration_file)
        nav2_navigation_configuration['map_server']['ros__parameters']['yaml_filename'] = self.map_info_file_path
        nav2_navigation_configuration['bt_navigator']['ros__parameters']['default_bt_xml_filename'] = behaviour_tree_configuration_path
        if robot_model == 'turtlebot3_waffle_performance_modelling':
            nav2_navigation_configuration['local_costmap']['local_costmap']['ros__parameters']['robot_radius'] = turtlebot_radius
            nav2_navigation_configuration['global_costmap']['global_costmap']['ros__parameters']['robot_radius'] = turtlebot_radius
        elif robot_model == 'hunter2':
            nav2_navigation_configuration['local_costmap']['local_costmap']['ros__parameters']['footprint'] = hunter2_footprint_string
            nav2_navigation_configuration['global_costmap']['global_costmap']['ros__parameters']['footprint'] = hunter2_footprint_string
        else:
            raise ValueError()
        if not path.exists(path.dirname(self.nav2_navigation_configuration_path)):
            os.makedirs(path.dirname(self.nav2_navigation_configuration_path))
        with open(self.nav2_navigation_configuration_path, 'w') as nav2_navigation_configuration_file:
            yaml.dump(nav2_navigation_configuration, nav2_navigation_configuration_file, default_flow_style=False)

        # copy the configuration of the behaviour_tree to the run folder
        if not path.exists(path.dirname(behaviour_tree_configuration_path)):
            os.makedirs(path.dirname(behaviour_tree_configuration_path))
        shutil.copyfile(original_behaviour_tree_configuration_path, behaviour_tree_configuration_path)

        # copy the configuration of global_planner to the run folder and update its parameters
        with open(original_global_planner_configuration_path) as global_planner_configuration_file:
            global_planner_configuration = yaml.safe_load(global_planner_configuration_file)
        if global_planner_node == 'smac':
            global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = min_turning_radius
        if not path.exists(path.dirname(self.global_planner_configuration_path)):
            os.makedirs(path.dirname(self.global_planner_configuration_path))
        with open(self.global_planner_configuration_path, 'w') as global_planner_configuration_file:
            yaml.dump(global_planner_configuration, global_planner_configuration_file, default_flow_style=False)

        # copy the configuration of local_planner to the run folder and update its parameters
        with open(original_local_planner_configuration_path) as local_planner_configuration_file:
            local_planner_configuration = yaml.safe_load(local_planner_configuration_file)
        if local_planner_node == 'teb':
            local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['min_turning_radius'] = min_turning_radius
            if robot_model == 'turtlebot3_waffle_performance_modelling':
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.type'] = "circular"
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.radius'] = turtlebot_radius
            elif robot_model == 'hunter2':
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.type'] = "polygon"
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.vertices'] = hunter2_footprint_string
            else:
                raise ValueError()
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

        # copy the configuration of the robot urdf to the run folder and update the link names for realistic data
        robot_realistic_urdf_tree = et.parse(original_robot_urdf_path)
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
            # 'nav2_amcl': nav2_amcl_configuration_relative_path,
            'nav2_navigation': nav2_navigation_configuration_relative_path,
            'behaviour_tree': behaviour_tree_configuration_relative_path,
            'gazebo_world_model': gazebo_world_model_relative_path,
            'gazebo_robot_model_sdf': gazebo_robot_model_sdf_relative_path,
            'gazebo_robot_model_config': gazebo_robot_model_config_relative_path,
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
        self.log(event=f"run_start")

        supervisor_params = {
            'configuration': self.supervisor_configuration_path,
            'use_sim_time': self.use_sim_time
        }

        environment_params = {
            'urdf': self.robot_realistic_urdf_path,
            'world': self.gazebo_world_model_path,
            'headless': self.headless,
            'gazebo_model_path_env_var': self.gazebo_model_path_env_var,
            'gazebo_plugin_path_env_var': self.gazebo_plugin_path_env_var,
            'params_file': self.nav2_navigation_configuration_path,
            'rviz_config_file': self.original_rviz_configuration_path
        }

        navigation_params = {
            'local_planner_params_file': self.local_planner_configuration_path,
            'global_planner_params_file': self.global_planner_configuration_path,
            'nav_params_file': self.nav2_navigation_configuration_path,
        }

        # declare components
        supervisor = Component('supervisor', 'local_planning_performance_modelling', 'local_planning_benchmark_supervisor.launch.py', supervisor_params)
        environment = Component('environment', 'local_planning_performance_modelling', 'environment.launch.py', environment_params)
        navigation = Component('navigation', 'local_planning_performance_modelling', 'navigation.launch.py', navigation_params)

        # add components to launcher
        components_launcher = ComponentsLauncher()
        # recorder.launch()  # TODO
        components_launcher.add_component(supervisor)
        components_launcher.add_component(environment)
        components_launcher.add_component(navigation)

        # launch components and wait for the supervisor to finish
        self.log(event="waiting_supervisor_finish")
        components_launcher.launch()
        self.log(event="supervisor_shutdown")

        # make sure remaining components have shutdown
        components_launcher.shutdown()
        print_info("execute_run: components shutdown completed")

        self.log(event="run_end")
        print_info(f"run {self.run_id} completed")
