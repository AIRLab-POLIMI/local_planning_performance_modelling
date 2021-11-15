#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import shutil

import yaml

import xml.etree.ElementTree as et

import time
from os import path
import numpy as np

from performance_modelling_py.benchmark_execution.log_software_versions import log_packages_and_repos
from performance_modelling_py.utils import backup_file_if_exists, print_info, print_error
from performance_modelling_py.component_proxies.ros2_component import Component, ComponentsLauncher


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

        amcl_alpha_1, amcl_alpha_2, amcl_alpha_3, amcl_alpha_4 = [None]*4
        localization_generator_update_rate = None
        localization_generator_translation_error = None
        localization_generator_rotation_error = None
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

        max_circumscribing_circle_radius = max(float(np.max(np.fabs(np.array(turtlebot_footprint)))),
                                               float(np.max(np.fabs(np.array(hunter2_footprint)))))  # largest radius for any robot's circumscribing circle
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
        self.run_completed_file_path = path.join(self.run_output_folder, "RUN_COMPLETED")
        backup_file_if_exists(self.run_output_folder)
        os.mkdir(self.run_output_folder)
        os.mkdir(run_configuration_path)
        self.recorder_output_path = path.join(self.run_output_folder, "all_data_rosbag2_record")
        self.ros_log_directory = path.join(self.run_output_folder, "logs")
        os.mkdir(self.ros_log_directory)

        # components original configuration paths
        components_configurations_folder = path.expanduser(self.benchmark_configuration['components_configurations_folder'])
        original_supervisor_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['supervisor'])
        original_localization_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][self.localization_node])
        original_nav2_navigation_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['nav2_navigation'])
        original_behaviour_tree_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['behaviour_tree'])
        self.original_rviz_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['rviz'])
        original_local_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][local_planner_node])
        original_global_planner_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration'][global_planner_node])
        original_gazebo_world_model_path = path.join(self.environment_folder, "gazebo", "gazebo_environment.model")
        original_gazebo_robot_model_config_path = path.join(robots_dataset_folder, robot_model, "model.config")
        original_gazebo_robot_model_sdf_path = path.join(robots_dataset_folder, robot_model, "model.sdf")
        original_robot_urdf_path = path.join(robots_dataset_folder, robot_model, "robot.urdf")

        # components configuration relative paths
        supervisor_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['supervisor'])
        localization_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration'][self.localization_node])
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
        self.localization_configuration_path = path.join(self.run_output_folder, localization_configuration_relative_path)
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

        # copy the configuration of the localization to the run folder and update its parameters
        with open(original_localization_configuration_path) as localization_configuration_file:
            localization_configuration = yaml.safe_load(localization_configuration_file)
        if self.localization_node == 'amcl':
            localization_configuration['amcl']['ros__parameters']['alpha1'] = amcl_alpha_1
            localization_configuration['amcl']['ros__parameters']['alpha2'] = amcl_alpha_2
            localization_configuration['amcl']['ros__parameters']['alpha3'] = amcl_alpha_3
            localization_configuration['amcl']['ros__parameters']['alpha4'] = amcl_alpha_4
        elif self.localization_node == 'localization_generator':
            localization_configuration['localization_generator']['ros__parameters']['update_pose_rate'] = localization_generator_update_rate
            localization_configuration['localization_generator']['ros__parameters']['translation_error'] = localization_generator_translation_error
            localization_configuration['localization_generator']['ros__parameters']['rotation_error'] = localization_generator_rotation_error
        else:
            raise ValueError()
        if not path.exists(path.dirname(self.localization_configuration_path)):
            os.makedirs(path.dirname(self.localization_configuration_path))
        with open(self.localization_configuration_path, 'w') as localization_configuration_file:
            yaml.dump(localization_configuration, localization_configuration_file, default_flow_style=False)

        # copy the configuration of the nav2_navigation to the run folder and update its parameters
        with open(original_nav2_navigation_configuration_path) as nav2_navigation_configuration_file:
            nav2_navigation_configuration = yaml.safe_load(nav2_navigation_configuration_file)
        nav2_navigation_configuration['map_server']['ros__parameters']['yaml_filename'] = self.map_info_file_path
        nav2_navigation_configuration['bt_navigator']['ros__parameters']['default_bt_xml_filename'] = behaviour_tree_configuration_path
        if robot_model == 'turtlebot3_waffle_performance_modelling':
            nav2_navigation_configuration['local_costmap']['local_costmap']['ros__parameters']['footprint'] = turtlebot_footprint_string
            nav2_navigation_configuration['global_costmap']['global_costmap']['ros__parameters']['footprint'] = turtlebot_footprint_string
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
            if robot_model == 'turtlebot3_waffle_performance_modelling':
                global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = turtlebot_min_turning_radius
            elif robot_model == 'hunter2':
                global_planner_configuration['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = hunter2_min_turning_radius
            else:
                raise ValueError()
        if not path.exists(path.dirname(self.global_planner_configuration_path)):
            os.makedirs(path.dirname(self.global_planner_configuration_path))
        with open(self.global_planner_configuration_path, 'w') as global_planner_configuration_file:
            yaml.dump(global_planner_configuration, global_planner_configuration_file, default_flow_style=False)

        # copy the configuration of local_planner to the run folder and update its parameters
        with open(original_local_planner_configuration_path) as local_planner_configuration_file:
            local_planner_configuration = yaml.safe_load(local_planner_configuration_file)
        if local_planner_node == 'teb':
            if robot_model == 'turtlebot3_waffle_performance_modelling':
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['cmd_angle_instead_rotvel'] = False
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.type'] = "polygon"
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.vertices'] = turtlebot_footprint_string
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['wheelbase'] = turtlebot_wheelbase
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['min_turning_radius'] = turtlebot_min_turning_radius
            elif robot_model == 'hunter2':
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['cmd_angle_instead_rotvel'] = True
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.type'] = "polygon"
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['footprint_model.vertices'] = hunter2_footprint_string
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['wheelbase'] = hunter2_wheelbase
                local_planner_configuration['controller_server']['ros__parameters']['FollowPath']['min_turning_radius'] = hunter2_min_turning_radius
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
        if alpha_1 == 0 and alpha_2 == 0 and alpha_3 == 0 and alpha_4 == 0:
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/odometry_source")[0].text = "1"
        else:
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/odometry_source")[0].text = "2"
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/alpha1")[0].text = str(alpha_1)
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/alpha2")[0].text = str(alpha_2)
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/alpha3")[0].text = str(alpha_3)
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/alpha4")[0].text = str(alpha_4)

        if robot_model == 'hunter2':
            gazebo_robot_model_sdf_root.findall(f".//plugin[@name='{robot_drive_plugin_type}']/max_steer")[0].text = str(max_steering_rad*1.1)
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
        run_info_dict["environment_folder"] = self.environment_folder
        run_info_dict["run_parameters"] = self.run_parameters
        run_info_dict["local_components_configuration"] = {
            'supervisor': supervisor_configuration_relative_path,
            'localization': localization_configuration_relative_path,
            'nav2_navigation': nav2_navigation_configuration_relative_path,
            'local_planner_configuration_relative_path': local_planner_configuration_relative_path,
            'global_planner_configuration_relative_path': global_planner_configuration_relative_path,
            'behaviour_tree': behaviour_tree_configuration_relative_path,
            'gazebo_world_model': gazebo_world_model_relative_path,
            'gazebo_robot_model_sdf': gazebo_robot_model_sdf_relative_path,
            'gazebo_robot_model_config': gazebo_robot_model_config_relative_path,
            'robot_realistic_urdf': robot_realistic_urdf_relative_path,
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

        print_info(f"t: {t}, run: {self.run_id}, event: {event}")
        try:
            with open(self.benchmark_log_path, 'a') as output_file:
                output_file.write(f"{t}, {self.run_id}, {event}\n")
        except IOError as e:
            print_error(f"benchmark_log: could not write event to file: {t}, {self.run_id}, {event}")
            print_error(e)

    def execute_run(self):
        self.log(event=f"run_start")

        # declare components
        supervisor = Component('supervisor', 'local_planning_performance_modelling', 'local_planning_benchmark_supervisor.launch.py', {
            'configuration': self.supervisor_configuration_path,
            'log_path': self.ros_log_directory,
        })
        recorder = Component('recorder', 'local_planning_performance_modelling', 'recorder.launch.py', {
            'recorder_output_path': self.recorder_output_path,
            'log_path': self.ros_log_directory,
        })
        environment = Component('environment', 'local_planning_performance_modelling', 'environment.launch.py', {
            'urdf': self.robot_realistic_urdf_path,
            'world': self.gazebo_world_model_path,
            'launch_rviz': self.launch_rviz,
            'launch_gzclient': self.launch_gzclient,
            'gazebo_model_path_env_var': self.gazebo_model_path_env_var,
            'gazebo_plugin_path_env_var': self.gazebo_plugin_path_env_var,
            'params_file': self.nav2_navigation_configuration_path,
            'rviz_config_file': self.original_rviz_configuration_path,
            'log_path': self.ros_log_directory,
        })
        navigation = Component('navigation', 'local_planning_performance_modelling', 'navigation.launch.py', {
            'local_planner_params_file': self.local_planner_configuration_path,
            'global_planner_params_file': self.global_planner_configuration_path,
            'nav_params_file': self.nav2_navigation_configuration_path,
            'log_path': self.ros_log_directory,
        })
        localization = Component('localization', 'local_planning_performance_modelling', f'{self.localization_node}.launch.py', {
            'localization_params_file': self.localization_configuration_path,
            'log_path': self.ros_log_directory,
        })

        # add components to launcher
        components_launcher = ComponentsLauncher()
        components_launcher.add_component(supervisor)
        if not self.do_not_record:
            components_launcher.add_component(recorder)
        components_launcher.add_component(environment)
        components_launcher.add_component(localization)
        components_launcher.add_component(navigation)

        # launch components and wait for the supervisor to finish
        self.log(event="waiting_supervisor_finish")
        components_launcher.launch()
        self.log(event="supervisor_shutdown")

        # make sure remaining components have shutdown
        components_launcher.shutdown()

        with open(self.run_completed_file_path, 'w') as _:
            pass

        self.log(event="run_end")
