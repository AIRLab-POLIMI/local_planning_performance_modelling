#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob
import pickle
from collections import defaultdict
from os import path
import numpy as np
import pandas as pd
import math
import warnings
import yaml
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2

from laser_geometry import LaserProjection
import shapely.geometry as shp

from performance_modelling_py.metrics.localization_metrics import get_matrix_diff
from performance_modelling_py.utils import print_error, print_info


class InterpolationException(Exception):
    pass


def interpolate_pose_2d_trajectories(trajectory_a_df, trajectory_b_df, trajectory_a_label='a', trajectory_b_label='b', average_rate=1.0, interpolation_tolerance=0.1, limit_trajectory_a_rate=True):
    a = trajectory_a_label
    b = trajectory_b_label

    trajectory_a_df['t_datetime'] = pd.to_datetime(trajectory_a_df['t'], unit='s')
    trajectory_b_df['t_datetime'] = pd.to_datetime(trajectory_b_df['t'], unit='s')

    trajectory_a_df_len = len(trajectory_a_df['t'])
    if trajectory_a_df_len < 2:
        raise InterpolationException(f"trajectory {a} has less than 2 poses")

    if limit_trajectory_a_rate:
        trajectory_a_rate = trajectory_a_df_len/(trajectory_a_df['t'][trajectory_a_df_len - 1] - trajectory_a_df['t'][0])
        trajectory_a_rate_limited_df = trajectory_a_df.iloc[::max(1, int(trajectory_a_rate / average_rate)), :]
    else:
        trajectory_a_rate_limited_df = trajectory_a_df

    # interpolate trajectory_b around trajectory_a timestamps
    tolerance = pd.Timedelta('{}s'.format(interpolation_tolerance))
    forward_matches = pd.merge_asof(
        left=trajectory_a_rate_limited_df[['t_datetime', 't', 'x', 'y', 'theta']],
        right=trajectory_b_df[['t_datetime', 't', 'x', 'y', 'theta']],
        on='t_datetime',
        direction='forward',
        tolerance=tolerance,
        suffixes=(f'_{a}', f'_{b}'))
    backward_matches = pd.merge_asof(
        left=trajectory_a_rate_limited_df[['t_datetime', 't', 'x', 'y', 'theta']],
        right=trajectory_b_df[['t_datetime', 't', 'x', 'y', 'theta']],
        on='t_datetime',
        direction='backward',
        tolerance=tolerance,
        suffixes=(f'_{a}', f'_{b}'))
    forward_backward_matches = pd.merge(
        left=backward_matches,
        right=forward_matches,
        on=f't_{a}')

    interpolated_trajectory_b_list = list()
    for index, row in forward_backward_matches.iterrows():
        t_b_1, t_b_2 = row[f't_{b}_x'], row[f't_{b}_y']
        t_int = row[f't_{a}']

        # if the trajectory_a time is too far from a trajectory_b time (before or after), do not use this trajectory_a data point
        if pd.isnull(t_b_1) or pd.isnull(t_b_2):
            continue

        x_a = row[f'x_{a}_x']
        x_b_1, x_b_2 = row[f'x_{b}_x'], row[f'x_{b}_y']
        x_int = np.interp(t_int, [t_b_1, t_b_2], [x_b_1, x_b_2])

        y_a = row[f'y_{a}_x']
        y_b_1, y_b_2 = row[f'y_{b}_x'], row[f'y_{b}_y']
        y_int = np.interp(t_int, [t_b_1, t_b_2], [y_b_1, y_b_2])

        theta_a = row[f'theta_{a}_x']
        theta_b_1, theta_b_2 = row[f'theta_{b}_x'], row[f'theta_{b}_y']
        theta_int = np.interp(t_int, [t_b_1, t_b_2], [theta_b_1, theta_b_2])

        interpolated_trajectory_b_list.append({
            't': t_int,
            f'x_{a}': x_a,
            f'y_{a}': y_a,
            f'theta_{a}': theta_a,
            f'x_{b}': x_int,
            f'y_{b}': y_int,
            f'theta_{b}': theta_int,
        })

    return pd.DataFrame(interpolated_trajectory_b_list)


def relative_2d_pose_transform(pose_a_1, pose_a_2, pose_b_1, pose_b_2):
    # compute transform from pose_a_1 to pose_a_2
    pose_a_transform_hc = get_matrix_diff(pose_a_1, pose_a_2)
    a_relative_pose = np.array([pose_a_transform_hc[0, 2], pose_a_transform_hc[1, 2], np.arctan2(pose_a_transform_hc[1, 0], pose_a_transform_hc[0, 0])])

    # compute transform from pose_b_1 to pose_b_2
    pose_b_transform_hc = get_matrix_diff(pose_b_1, pose_b_2)
    b_relative_pose = np.array([pose_b_transform_hc[0, 2], pose_b_transform_hc[1, 2], np.arctan2(pose_b_transform_hc[1, 0], pose_b_transform_hc[0, 0])])

    # compute relative transform from a to b
    relative_transform_hc = get_matrix_diff(a_relative_pose, b_relative_pose)
    x, y, theta = relative_transform_hc[0, 2], relative_transform_hc[1, 2], np.arctan2(relative_transform_hc[1, 0], relative_transform_hc[0, 0])
    return x, y, theta


class TrajectoryLength:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "trajectory_length"
        self.version = 1

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error(f"{self.metric_name}: ground_truth_poses file not found:\n{self.ground_truth_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df[self.metric_name] = [np.nan]

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for ground truth poses
        ground_truth_poses_df = pd.read_csv(self.ground_truth_poses_file_path)

        ground_truth_poses_df_clipped = ground_truth_poses_df[(navigation_start_time <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= navigation_end_time)]
        ground_truth_positions = ground_truth_poses_df_clipped[['x', 'y']].values
        # fai una prova con 4 coordinate per vedere se usare atan o atan2 (mi aspetto 0)
        squared_deltas = (ground_truth_positions[1:-1] - ground_truth_positions[0:-2]) ** 2  # equivalent to (x_2-x_1)**2, (y_2-y_1)**2, for each row
        sum_of_squared_deltas = np.sum(squared_deltas, axis=1)  # equivalent to (x_2-x_1)**2 + (y_2-y_1)**2, for each row
        euclidean_distance_of_deltas = np.sqrt(sum_of_squared_deltas)  # equivalent to sqrt( (x_2-x_1)**2 + (y_2-y_1)**2 ), for each row
        trajectory_length = euclidean_distance_of_deltas.sum()

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df[self.metric_name] = [float(trajectory_length)]
        return True


class ExecutionTime:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "execution_time"
        self.version = 1

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df[self.metric_name] = [np.nan]

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t
        execution_time = navigation_end_time - navigation_start_time

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df[self.metric_name] = [float(execution_time)]
        return True


class SuccessRate:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "success_rate"
        self.version = 1

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df[self.metric_name] = [np.nan]

        # get events info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_goal_reached_events = run_events_df[run_events_df.event == 'navigation_goal_reached']

        navigation_goal_reached = len(navigation_goal_reached_events) == 1
        #print("ngr: " + str(navigation_goal_reached))
        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df[self.metric_name] = [int(navigation_goal_reached)]
        return True


class CollisionRate:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.scans_file_path = path.join(run_output_folder, "benchmark_data", "scans.csv")
        self.local_costmap_params_path = path.join(run_output_folder, "components_configuration", "navigation_stack", "navigation.yaml")
        self.run_info_path = path.join(run_output_folder, "run_info.yaml")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "collision_rate"
        self.version = 3

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        if not path.isfile(self.scans_file_path):
            print_error(f"{self.metric_name}: scans_file file not found:\n{self.scans_file_path}")
            return False

        if not path.isfile(self.local_costmap_params_path):
            print_error(f"{self.metric_name}: local_costmap_params file not found:\n{self.local_costmap_params_path}")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df['collision_rate'] = [np.nan]
        self.results_df['collision_time'] = [np.nan]

        with open(self.local_costmap_params_path) as local_costmap_params_file:
            local_costmap_params = yaml.safe_load(local_costmap_params_file)
        footprint_points_list = yaml.safe_load(local_costmap_params['local_costmap']['footprint'])
        footprint_polygon = shp.Polygon(footprint_points_list)

        # get base_scan offset for this robot
        with open(self.run_info_path) as run_info_file:
            run_info = yaml.safe_load(run_info_file)
        robot_model_name = run_info['run_parameters']['robot_model']
        if robot_model_name == 'turtlebot3_waffle_performance_modelling':  # TODO avoid hardcoding the offsets here
            base_scan_x_offset = -0.064
            base_scan_y_offset = 0.0
        elif robot_model_name == 'hunter2':
            base_scan_x_offset = 0.325
            base_scan_y_offset = 0.115
        else:
            print_error(f"{self.metric_name}: robot_model_name not valid:\n{robot_model_name}")
            return False

        # get events info from run events
        scans_df = pd.read_csv(self.scans_file_path, engine='python', sep=', ')

        collision = False
        collision_time = np.nan
        for i, scan_row in scans_df.iterrows():
            laser_scan_msg = LaserScan()
            laser_scan_msg.angle_min = float(scan_row[1])
            laser_scan_msg.angle_max = float(scan_row[2])
            laser_scan_msg.angle_increment = float(scan_row[3])
            laser_scan_msg.range_min = float(scan_row[4])
            laser_scan_msg.range_max = float(scan_row[5])
            laser_scan_msg.ranges = list(map(float, scan_row[6:]))
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                pointcloud_msg = LaserProjection().projectLaser(laser_scan_msg)
            point_generator = pc2.read_points(pointcloud_msg)
            for point_pc2 in point_generator:
                if not np.isnan(point_pc2[0]):
                    point = shp.Point(point_pc2[0] + base_scan_x_offset, point_pc2[1] + base_scan_y_offset)
                    if footprint_polygon.contains(point):
                        collision = True
                        collision_time = float(scan_row[0])
                        break
            if collision:
                break

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df['collision_rate'] = [int(collision)]
        self.results_df['collision_time'] = [float(collision_time)]
        return True


class Clearance:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.scans_file_path = path.join(run_output_folder, "benchmark_data", "scans.csv")
        self.local_costmap_params_path = path.join(run_output_folder, "components_configuration", "navigation_stack", "navigation.yaml")
        self.run_info_path = path.join(run_output_folder, "run_info.yaml")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "clearance"
        self.version = 3
    
    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        if not path.isfile(self.scans_file_path):
            print_error(f"{self.metric_name}: scans_file file not found:\n{self.scans_file_path}")
            return False

        if not path.isfile(self.local_costmap_params_path):
            print_error(f"{self.metric_name}: local_costmap_params file not found:\n{self.local_costmap_params_path}")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df['minimum_clearance'] = [np.nan]
        self.results_df['average_clearance'] = [np.nan]
        self.results_df['median_clearance']  = [np.nan]
        self.results_df['maximum_clearance'] = [np.nan]

        with open(self.local_costmap_params_path) as local_costmap_params_file:
            local_costmap_params = yaml.safe_load(local_costmap_params_file)
        footprint_points_list = yaml.safe_load(local_costmap_params['local_costmap']['footprint'])
        footprint_polygon = shp.Polygon(footprint_points_list)

        # get base_scan offset for this robot
        with open(self.run_info_path) as run_info_file:
            run_info = yaml.safe_load(run_info_file)
        robot_model_name = run_info['run_parameters']['robot_model']
        if robot_model_name == 'turtlebot3_waffle_performance_modelling':  # TODO avoid hardcoding the offsets here
            base_scan_x_offset = -0.064
            base_scan_y_offset = 0.0
        elif robot_model_name == 'hunter2':
            base_scan_x_offset = 0.325
            base_scan_y_offset = 0.115
        else:
            print_error(f"{self.metric_name}: robot_model_name not valid:\n{robot_model_name}")
            return False

        # get events info from run events
        scans_df = pd.read_csv(self.scans_file_path, engine='python', sep=', ')

        clearance_list = list()
        
        for i, scan_row in scans_df.iterrows():
            laser_scan_msg = LaserScan()
            laser_scan_msg.angle_min = float(scan_row[1])
            laser_scan_msg.angle_max = float(scan_row[2])
            laser_scan_msg.angle_increment = float(scan_row[3])
            laser_scan_msg.range_min = float(scan_row[4])
            laser_scan_msg.range_max = float(scan_row[5])
            laser_scan_msg.ranges = list(map(float, scan_row[6:]))
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                pointcloud_msg = LaserProjection().projectLaser(laser_scan_msg)
            point_generator = pc2.read_points(pointcloud_msg)
            points_clearance_list = list()
            for point_pc2 in point_generator:
                if not np.isnan(point_pc2[0]):
                    point = shp.Point(point_pc2[0] + base_scan_x_offset, point_pc2[1] + base_scan_y_offset)
                    dist = point.distance(footprint_polygon) if not footprint_polygon.contains(point) else 0.0
                    points_clearance_list.append(dist)

            if (len(points_clearance_list) > 0): # compute the min only if the list is not empty
                clearance_list.append(np.min(points_clearance_list))

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df['minimum_clearance'] = [float(np.min(clearance_list))]
        self.results_df['average_clearance'] = [float(np.mean(clearance_list))]
        self.results_df['median_clearance'] = [float(np.median(clearance_list))]
        self.results_df['maximum_clearance'] = [float(np.max(clearance_list))]
        return True


class CpuTimeAndMaxMemoryUsage:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ps_snapshots_folder_path = path.join(run_output_folder, "benchmark_data", "ps_snapshots")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "cpu_time_and_max_memory"
        self.version = 6

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["move_base_cpu_time"] = [np.nan]
        self.results_df["simulation_cpu_time"] = [np.nan]
        self.results_df["system_cpu_time"] = [np.nan]
        self.results_df["move_base_max_memory"] = [np.nan]
        self.results_df["simulation_max_memory"] = [np.nan]
        self.results_df["system_max_memory"] = [np.nan]

        # check required files exist
        if not path.isdir(self.ps_snapshots_folder_path):
            print_info(f"{self.metric_name}: ps_snapshots directory not found:\n{self.ps_snapshots_folder_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        ps_snapshot_files_path = path.join(self.ps_snapshots_folder_path, "ps_*.pkl")
        ps_snapshot_paths_list = sorted(glob.glob(ps_snapshot_files_path))
        if len(ps_snapshot_paths_list) == 0:
            print_info(f"{self.metric_name}: ps_snapshot files not found:\n{ps_snapshot_files_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        system_cpu_time_dict = defaultdict(int)
        system_max_memory_dict = defaultdict(int)

        simulation_cpu_time_dict = defaultdict(int)
        simulation_max_memory_dict = defaultdict(int)

        for ps_snapshot_path in ps_snapshot_paths_list:
            try:
                with open(ps_snapshot_path, 'rb') as ps_snapshot_file:
                    ps_snapshot = pickle.load(ps_snapshot_file)
            except (EOFError, pickle.UnpicklingError) as e:
                print_error(f"{self.metric_name}: Could not load pickled ps snapshot. Error: {type(e)} {e}. Pickle file:\n{ps_snapshot_path}")
                continue
            for process_info in ps_snapshot:
                process_name = process_info['name']
                if process_name is None:
                    print_info(f"{self.metric_name}: found process {process_name}. Aborting.\n")
                    return True
                if process_info['cpu_times'] is not None and process_info['memory_full_info'] is not None:
                    if process_name in ['gzserver', 'gzclient', 'rviz', 'pedsim_simulator', 'pedsim_visualizer_node', 'python', 'record']:  # consider simulator and rviz to count the robot system memory. python is the name of 2 processes: the supervisor and spawn_pedsim_agents
                            simulation_cpu_time_dict[process_name] = max(
                                simulation_cpu_time_dict[process_name],
                                process_info['cpu_times'].user + process_info['cpu_times'].system
                            )
                            simulation_max_memory_dict[process_name] = max(
                                simulation_max_memory_dict[process_name],
                                process_info['memory_full_info'].pss
                            )
                    else:
                        if process_info['cpu_times'] is not None and process_info['memory_full_info'] is not None:
                            system_cpu_time_dict[process_name] = max(
                                system_cpu_time_dict[process_name],
                                process_info['cpu_times'].user + process_info['cpu_times'].system
                            )
                            system_max_memory_dict[process_name] = max(
                                system_max_memory_dict[process_name],
                                process_info['memory_full_info'].pss
                            )
                else:
                    pass  # sometimes this happens when the process terminates in the previous snapshot
                    

        if len(system_cpu_time_dict) == 0 or len(system_max_memory_dict) == 0:
            print_error(f"{self.metric_name}: no data from ps snapshots:\n{ps_snapshot_files_path}")
            return False

        self.results_df["move_base_cpu_time"] = [system_cpu_time_dict["move_base"]]
        self.results_df["simulation_cpu_time"] = [sum(simulation_cpu_time_dict.values())]
        self.results_df["system_cpu_time"] = [sum(system_cpu_time_dict.values())]

        self.results_df["move_base_max_memory"] = [system_max_memory_dict["move_base"]]
        self.results_df["simulation_max_memory"] = [sum(simulation_max_memory_dict.values())]
        self.results_df["system_max_memory"] = [sum(system_max_memory_dict.values())]

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class Dispersion: 
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.scans_file_path = path.join(run_output_folder, "benchmark_data", "scans.csv")
        self.local_costmap_params_path = path.join(run_output_folder, "components_configuration", "navigation_stack", "navigation.yaml")
        self.run_info_path = path.join(run_output_folder, "run_info.yaml")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "dispersion"
        self.version = 1

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        if not path.isfile(self.scans_file_path):
            print_error(f"{self.metric_name}: scans_file file not found:\n{self.scans_file_path}")
            return False

        if not path.isfile(self.local_costmap_params_path):
            print_error(f"{self.metric_name}: local_costmap_params file not found:\n{self.local_costmap_params_path}")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df['dispersion'] = [np.nan]

        with open(self.local_costmap_params_path) as local_costmap_params_file:
            local_costmap_params = yaml.safe_load(local_costmap_params_file)
        footprint_points_list = yaml.safe_load(local_costmap_params['local_costmap']['footprint'])
        footprint_polygon = shp.Polygon(footprint_points_list)

        # get base_scan offset for this robot
        with open(self.run_info_path) as run_info_file:
            run_info = yaml.safe_load(run_info_file)
        robot_model_name = run_info['run_parameters']['robot_model']
        if robot_model_name == 'turtlebot3_waffle_performance_modelling':  # TODO avoid hardcoding the offsets here
            base_scan_x_offset = -0.064
            base_scan_y_offset = 0.0
        elif robot_model_name == 'hunter2':
            base_scan_x_offset = 0.325
            base_scan_y_offset = 0.115
        else:
            print_error(f"{self.metric_name}: robot_model_name not valid:\n{robot_model_name}")
            return False

        # get events info from run events
        scans_df = pd.read_csv(self.scans_file_path, engine='python', sep=', ')

        # opzione 1: usa numpy per fare la media dei 360 raggi e ottenere una lista da 16 numpy.mean
        # opzione 2: scartare valori cambiando angle minx, max e angle increment
        local_dispersion_list = list()  # lista in cui aggiungo in ogni cella la dispersione calcolata in un determinato punto del path.
        for i, scan_row in scans_df.iterrows():
            laser_scan_msg = LaserScan()
            laser_scan_msg.angle_min = float(scan_row[1])
            laser_scan_msg.angle_max = float(scan_row[2])
            laser_scan_msg.angle_increment = float(scan_row[3])
            laser_scan_msg.range_min = float(scan_row[4])   # distanza minima a cui l'ostacolo deve essere per poter essere rilevato dallo scan
            laser_scan_msg.range_max = float(scan_row[5])   # distanza massima a cui l'ostacolo deve essere per poter essere rilevato dallo scan
            laser_scan_msg.ranges = list(map(float, scan_row[6:]))
        #opzione 1: creare gridmap dal laser (matrice con 0 e 1 con numpy)
        #opzione 2: discretizzare in 16 raggi per evitare errori dovuti al rumore sulla misurazione

        #mentre calcoli la metrica fai un plot dei 16 punti scatter plot plt.scatter (sia dei 360 che dei 16 per confrontare input con output)
            dispersion = 0
            i = 0
            while i < len(laser_scan_msg.ranges)-1: #sostituisci == float(inf) con <= r poi
                if (laser_scan_msg.ranges[i] == float("inf") and laser_scan_msg.ranges[i+1] != float("inf")) or (laser_scan_msg.ranges[i] != float("inf") and laser_scan_msg.ranges[i+1] == float("inf")): 
                    dispersion += 1
                i+=1
            
            local_dispersion_list.append(dispersion) # add to list each local dispersion

        global_dispersion = sum(local_dispersion_list) / len(local_dispersion_list)
        # print("Global dispersion: ", global_dispersion)

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df['dispersion'] = [float(global_dispersion)]
        return True

        
class OdometryError:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.odometry_poses_file_path = path.join(run_output_folder, "benchmark_data", "odom.csv")
        self.localization_update_poses_file_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "odometry_error"
        self.version = 2

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["odometry_error_alpha_1_mean"] = [np.nan]
        self.results_df["odometry_error_alpha_1_std"] = [np.nan]
        self.results_df["odometry_error_alpha_2_mean"] = [np.nan]
        self.results_df["odometry_error_alpha_2_std"] = [np.nan]
        self.results_df["odometry_error_alpha_3_mean"] = [np.nan]
        self.results_df["odometry_error_alpha_3_std"] = [np.nan]
        self.results_df["odometry_error_alpha_4_mean"] = [np.nan]
        self.results_df["odometry_error_alpha_4_std"] = [np.nan]

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error(f"{self.metric_name}: ground_truth_poses file not found:\n{self.ground_truth_poses_file_path}")
            return False

        if not path.isfile(self.odometry_poses_file_path):
            print_error(f"{self.metric_name}: odometry_poses file not found:\n{self.odometry_poses_file_path}")
            return False

        if not path.isfile(self.localization_update_poses_file_path):
            print_error(f"{self.metric_name}: estimated_localization_update_poses file not found:\n{self.localization_update_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for ground truth poses
        ground_truth_poses_df = pd.read_csv(self.ground_truth_poses_file_path)
        ground_truth_poses_df = ground_truth_poses_df[(navigation_start_time <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= navigation_end_time)]

        # get the dataframes for odom poses
        odom_poses_df = pd.read_csv(self.odometry_poses_file_path)
        if len(odom_poses_df) == 0:
            print_info(f"{self.metric_name}: not enough odom poses in navigation interval [{navigation_start_time}, {navigation_end_time}]:\n{self.odometry_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True
        odom_poses_df = odom_poses_df[(navigation_start_time <= odom_poses_df.t) & (odom_poses_df.t <= navigation_end_time)]

        # get the dataframes for localization update poses
        localization_update_poses_df = pd.read_csv(self.localization_update_poses_file_path)
        if len(localization_update_poses_df) == 0:
            if self.verbose:
                print_info(f"{self.metric_name}: no localization update poses in:\n{self.localization_update_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True
        localization_update_timestamps = localization_update_poses_df[(navigation_start_time <= localization_update_poses_df.t) & (localization_update_poses_df.t <= navigation_end_time)]['t'].values
        localization_update_timestamps_pairs = zip(list(localization_update_timestamps[0:-1]), list(localization_update_timestamps[1:]))

        # compute the interpolated ground truth poses
        try:
            interpolated_df = interpolate_pose_2d_trajectories(
                trajectory_a_df=odom_poses_df, trajectory_a_label='odom',
                trajectory_b_df=ground_truth_poses_df, trajectory_b_label='gt',
                limit_trajectory_a_rate=False, interpolation_tolerance=0.1
            )
        except InterpolationException as e:
            print_info(f"{self.metric_name}: interpolation exception: {e} when interpolating:\n{self.ground_truth_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        # if not enough matching ground truth data points are found, the metrics can not be computed
        if len(interpolated_df.index) < 2:
            print_info(f"{self.metric_name}: no matching ground truth data points were found when interpolating:\n{self.ground_truth_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        odom_errors_alpha_1 = list()
        odom_errors_alpha_2 = list()
        odom_errors_alpha_3 = list()
        odom_errors_alpha_4 = list()
        for start_timestamp, end_timestamp in localization_update_timestamps_pairs:
            assert (start_timestamp < end_timestamp)

            # get start pose and end pose that most closely matches start and end times
            interpolated_df_clipped = interpolated_df[(start_timestamp <= interpolated_df.t) & (interpolated_df.t <= end_timestamp)]
            if len(interpolated_df_clipped) < 1:
                if self.verbose:
                    print_info(f"{self.metric_name}: ground truth rate too low compared to localization update rate in update timestamps interval [{start_timestamp}, {end_timestamp}]:\n{self.ground_truth_poses_file_path}")
                continue
            if len(interpolated_df[(interpolated_df.t < start_timestamp)]) < 1:
                if self.verbose:
                    if self.verbose:
                        print_info(f"{self.metric_name}: not enough interpolated poses before update interval [{start_timestamp}, {end_timestamp}]:\n{self.ground_truth_poses_file_path}")
                continue
            interpolated_start_poses = interpolated_df[(interpolated_df.t < start_timestamp)].iloc[-1]
            interpolated_end_poses = interpolated_df_clipped.iloc[-1]

            # compute ground truth and odom transforms from start to end
            ground_truth_start_pose = interpolated_start_poses[['x_gt', 'y_gt', 'theta_gt']].values
            ground_truth_end_pose = interpolated_end_poses[['x_gt', 'y_gt', 'theta_gt']].values
            odom_start_pose = interpolated_start_poses[['x_odom', 'y_odom', 'theta_odom']].values
            odom_end_pose = interpolated_end_poses[['x_odom', 'y_odom', 'theta_odom']].values

            # compute relative transform between ground truth and odom
            relative_error_x, relative_error_y, relative_error_theta = relative_2d_pose_transform(ground_truth_start_pose, ground_truth_end_pose, odom_start_pose, odom_end_pose)
            relative_translation_error = np.sqrt(relative_error_x ** 2 + relative_error_y ** 2)
            relative_rotation_error = np.abs(np.arctan2(np.sin(relative_error_theta), np.cos(relative_error_theta)))

            # compute the integral of translation between updates
            ground_truth_poses_df_clipped = ground_truth_poses_df[(start_timestamp <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= end_timestamp)]
            ground_truth_positions = ground_truth_poses_df_clipped[['x', 'y']].values
            squared_deltas = (ground_truth_positions[1:-1] - ground_truth_positions[0:-2]) ** 2  # equivalent to (x_2-x_1)**2, (y_2-y_1)**2, for each row
            trajectory_translation = np.sqrt(np.sum(squared_deltas, axis=1)).sum()  # equivalent to sum for each row of sqrt( (x_2-x_1)**2 + (y_2-y_1)**2 )

            # compute the integral of rotation between updates
            ground_truth_rotations = ground_truth_poses_df_clipped['theta'].values
            rotation_differences = ground_truth_rotations[1:-1] - ground_truth_rotations[0:-2]
            rotation_deltas = np.abs(np.arctan2(np.sin(rotation_differences), np.cos(rotation_differences)))  # normalize the angle difference
            trajectory_rotation = np.sum(rotation_deltas)

            if trajectory_rotation > 0.01:  # only compute this errors if there was a meaningful rotation (0.01rad =~ 0.6deg)
                odom_errors_alpha_1.append(relative_rotation_error/trajectory_rotation)
                odom_errors_alpha_4.append(relative_translation_error/trajectory_rotation)
            if trajectory_translation > 0.01:  # only compute this errors if there was a meaningful translation (1cm)
                odom_errors_alpha_2.append(relative_rotation_error/trajectory_translation)
                odom_errors_alpha_3.append(relative_translation_error/trajectory_translation)

        if len(odom_errors_alpha_1) == 0 or len(odom_errors_alpha_2) == 0 or len(odom_errors_alpha_3) == 0 or len(odom_errors_alpha_4) == 0:
            print_error(f"{self.metric_name}: len(odom_errors_alpha_i) == 0:\n{self.ground_truth_poses_file_path}")

        self.results_df["odometry_error_alpha_1_mean"] = [float(np.mean(odom_errors_alpha_1)) if len(odom_errors_alpha_1) else np.nan]
        self.results_df["odometry_error_alpha_1_std"] = [float(np.std(odom_errors_alpha_1)) if len(odom_errors_alpha_1) else np.nan]
        self.results_df["odometry_error_alpha_2_mean"] = [float(np.mean(odom_errors_alpha_2)) if len(odom_errors_alpha_2) else np.nan]
        self.results_df["odometry_error_alpha_2_std"] = [float(np.std(odom_errors_alpha_2)) if len(odom_errors_alpha_2) else np.nan]
        self.results_df["odometry_error_alpha_3_mean"] = [float(np.mean(odom_errors_alpha_3)) if len(odom_errors_alpha_3) else np.nan]
        self.results_df["odometry_error_alpha_3_std"] = [float(np.std(odom_errors_alpha_3)) if len(odom_errors_alpha_3) else np.nan]
        self.results_df["odometry_error_alpha_4_mean"] = [float(np.mean(odom_errors_alpha_4)) if len(odom_errors_alpha_4) else np.nan]
        self.results_df["odometry_error_alpha_4_std"] = [float(np.std(odom_errors_alpha_4)) if len(odom_errors_alpha_4) else np.nan]
        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class LocalizationError:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.localization_update_poses_file_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.errors_plot_file_path = path.join(run_output_folder, "metric_results", "localization_errors.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "localization_update_error"
        self.version = 4

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["localization_update_absolute_translation_error_mean"] = [np.nan]
        self.results_df["localization_update_absolute_translation_error_std"] = [np.nan]
        self.results_df["localization_update_absolute_rotation_error_mean"] = [np.nan]
        self.results_df["localization_update_absolute_rotation_error_std"] = [np.nan]
        self.results_df["localization_update_relative_translation_error_mean"] = [np.nan]
        self.results_df["localization_update_relative_translation_error_std"] = [np.nan]
        self.results_df["localization_update_relative_rotation_error_mean"] = [np.nan]
        self.results_df["localization_update_relative_rotation_error_std"] = [np.nan]
        self.results_df["localization_update_normalized_relative_translation_error_mean"] = [np.nan]
        self.results_df["localization_update_normalized_relative_translation_error_std"] = [np.nan]
        self.results_df["localization_update_normalized_relative_rotation_error_mean"] = [np.nan]
        self.results_df["localization_update_normalized_relative_rotation_error_std"] = [np.nan]

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error(f"{self.metric_name}: ground_truth_poses file not found:\n{self.ground_truth_poses_file_path}")
            return False

        if not path.isfile(self.localization_update_poses_file_path):
            print_error(f"{self.metric_name}: estimated_localization_update_poses file not found:\n{self.localization_update_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for ground truth poses
        ground_truth_poses_df = pd.read_csv(self.ground_truth_poses_file_path)
        ground_truth_poses_df = ground_truth_poses_df[(navigation_start_time <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= navigation_end_time)]

        # get the dataframes for localization update poses
        localization_update_poses_df = pd.read_csv(self.localization_update_poses_file_path)
        if len(localization_update_poses_df) == 0:
            if self.verbose:
                if self.verbose:
                    print_info(f"{self.metric_name}: not enough localization update poses in navigation interval [{navigation_start_time}, {navigation_end_time}]:\n{self.localization_update_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True
        localization_update_poses_df = localization_update_poses_df[(navigation_start_time <= localization_update_poses_df.t) & (localization_update_poses_df.t <= navigation_end_time)]

        # compute the interpolated ground truth poses
        try:
            interpolated_df = interpolate_pose_2d_trajectories(
                trajectory_a_df=localization_update_poses_df, trajectory_a_label='est',
                trajectory_b_df=ground_truth_poses_df, trajectory_b_label='gt',
                limit_trajectory_a_rate=False, interpolation_tolerance=0.1
            )
        except InterpolationException as e:
            print_info(f"{self.metric_name}: interpolation exception: {e} when interpolating:\n{self.ground_truth_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        # if not enough matching ground truth data points are found, the metrics can not be computed
        if len(interpolated_df.index) < 2:
            print_info(f"{self.metric_name}: no matching ground truth data points were found when interpolating:\n{self.ground_truth_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        interpolated_poses = interpolated_df[['t', 'x_est', 'y_est', 'theta_est', 'x_gt', 'y_gt', 'theta_gt']].values

        times = list()
        trajectory_translations = list()
        trajectory_rotations = list()
        absolute_translation_errors = list()
        absolute_rotation_errors = list()
        relative_translation_errors = list()
        relative_rotation_errors = list()
        normalized_relative_translation_errors = list()
        normalized_relative_translation_w_nans_errors = list()
        normalized_relative_rotation_errors = list()
        normalized_relative_rotation_w_nans_errors = list()
        for i in range(1, len(interpolated_df)):
            start_timestamp = interpolated_poses[i - 1, 0]
            end_timestamp = interpolated_poses[i, 0]

            # compute ground truth and estimate transforms from start to end
            ground_truth_start_pose = interpolated_poses[i - 1, 4:7]
            x_gt, y_gt, theta_gt = ground_truth_end_pose = interpolated_poses[i, 4:7]
            est_start_pose = interpolated_poses[i - 1, 1:4]
            x_est, y_est, theta_est = est_end_pose = interpolated_poses[i, 1:4]

            # compute absolute error
            absolute_translation_error = np.sqrt((x_gt - x_est) ** 2 + (y_gt - y_est) ** 2)
            absolute_rotation_error = np.abs(np.arctan2(np.sin(theta_gt - theta_est), np.cos(theta_gt - theta_est)))

            # compute relative error
            relative_error_x, relative_error_y, relative_error_theta = relative_2d_pose_transform(ground_truth_start_pose, ground_truth_end_pose, est_start_pose, est_end_pose)
            relative_translation_error = np.sqrt(relative_error_x ** 2 + relative_error_y ** 2)
            relative_rotation_error = np.abs(np.arctan2(np.sin(relative_error_theta), np.cos(relative_error_theta)))

            # compute the integral of translation between updates
            ground_truth_poses_df_clipped = ground_truth_poses_df[(start_timestamp <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= end_timestamp)]
            ground_truth_positions = ground_truth_poses_df_clipped[['x', 'y']].values
            squared_deltas = (ground_truth_positions[1:-1] - ground_truth_positions[0:-2]) ** 2  # equivalent to (x_2-x_1)**2, (y_2-y_1)**2, for each row
            trajectory_translation = np.sqrt(np.sum(squared_deltas, axis=1)).sum()  # equivalent to sum for each row of sqrt( (x_2-x_1)**2 + (y_2-y_1)**2 )

            # compute the integral of rotation between updates
            ground_truth_rotations = ground_truth_poses_df_clipped['theta'].values
            rotation_differences = ground_truth_rotations[1:-1] - ground_truth_rotations[0:-2]
            rotation_deltas = np.abs(np.arctan2(np.sin(rotation_differences), np.cos(rotation_differences)))  # normalize the angle difference
            trajectory_rotation = np.sum(rotation_deltas)

            times.append(end_timestamp)
            absolute_translation_errors.append(absolute_translation_error)
            absolute_rotation_errors.append(absolute_rotation_error)

            relative_translation_errors.append(relative_translation_error)
            relative_rotation_errors.append(relative_rotation_error)

            trajectory_translations.append(trajectory_translation)
            trajectory_rotations.append(trajectory_rotation)

            if trajectory_translation > 0.01:  # only compute this errors if there was a meaningful translation (1cm)
                normalized_relative_translation_errors.append(relative_translation_error / trajectory_translation)
                normalized_relative_translation_w_nans_errors.append(relative_translation_error / trajectory_translation)
            else:
                normalized_relative_translation_w_nans_errors.append(np.nan)
            if trajectory_rotation > 0.05:  # only compute this errors if there was a meaningful rotation (0.05rad =~ 3deg)
                normalized_relative_rotation_errors.append(relative_rotation_error/trajectory_rotation)
                normalized_relative_rotation_w_nans_errors.append(relative_rotation_error/trajectory_rotation)
            else:
                normalized_relative_rotation_w_nans_errors.append(np.nan)

        errors_plot_df = pd.DataFrame({
            't': times,
            'l': trajectory_translations,
            'r': trajectory_rotations,
            'ate': absolute_translation_errors,
            'are': absolute_rotation_errors,
            'rte': relative_translation_errors,
            'rre': relative_rotation_errors,
            'nrte': normalized_relative_translation_w_nans_errors,
            'nrre': normalized_relative_rotation_w_nans_errors,
        })
        errors_plot_df.to_csv(self.errors_plot_file_path)

        self.results_df["localization_update_absolute_translation_error_mean"] = [float(np.mean(absolute_translation_errors)) if len(absolute_translation_errors) else np.nan]
        self.results_df["localization_update_absolute_translation_error_std"] = [float(np.std(absolute_translation_errors)) if len(absolute_translation_errors) else np.nan]
        self.results_df["localization_update_absolute_rotation_error_mean"] = [float(np.mean(absolute_rotation_errors)) if len(absolute_rotation_errors) else np.nan]
        self.results_df["localization_update_absolute_rotation_error_std"] = [float(np.std(absolute_rotation_errors)) if len(absolute_rotation_errors) else np.nan]
        self.results_df["localization_update_relative_translation_error_mean"] = [float(np.mean(relative_translation_errors)) if len(relative_translation_errors) else np.nan]
        self.results_df["localization_update_relative_translation_error_std"] = [float(np.std(relative_translation_errors)) if len(relative_translation_errors) else np.nan]
        self.results_df["localization_update_relative_rotation_error_mean"] = [float(np.mean(relative_rotation_errors)) if len(relative_rotation_errors) else np.nan]
        self.results_df["localization_update_relative_rotation_error_std"] = [float(np.std(relative_rotation_errors)) if len(relative_rotation_errors) else np.nan]
        self.results_df["localization_update_normalized_relative_translation_error_mean"] = [float(np.mean(normalized_relative_translation_errors)) if len(normalized_relative_translation_errors) else np.nan]
        self.results_df["localization_update_normalized_relative_translation_error_std"] = [float(np.std(normalized_relative_translation_errors)) if len(normalized_relative_translation_errors) else np.nan]
        self.results_df["localization_update_normalized_relative_rotation_error_mean"] = [float(np.mean(normalized_relative_rotation_errors)) if len(normalized_relative_rotation_errors) else np.nan]
        self.results_df["localization_update_normalized_relative_rotation_error_std"] = [float(np.std(normalized_relative_rotation_errors)) if len(normalized_relative_rotation_errors) else np.nan]

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class CollisionlessLocalizationError:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.localization_update_poses_file_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "collisionless_localization_update_error"
        self.version = 1

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        if 'collision_rate' not in self.results_df or np.isnan(self.results_df['collision_rate'].iloc[0]):
            print_error(f"{self.metric_name}: collision_rate result not found or nan")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["collisionless_localization_update_absolute_translation_error_mean"] = [np.nan]
        self.results_df["collisionless_localization_update_absolute_translation_error_std"] = [np.nan]
        self.results_df["collisionless_localization_update_absolute_rotation_error_mean"] = [np.nan]
        self.results_df["collisionless_localization_update_absolute_rotation_error_std"] = [np.nan]
        self.results_df["collisionless_localization_update_relative_translation_error_mean"] = [np.nan]
        self.results_df["collisionless_localization_update_relative_translation_error_std"] = [np.nan]
        self.results_df["collisionless_localization_update_relative_rotation_error_mean"] = [np.nan]
        self.results_df["collisionless_localization_update_relative_rotation_error_std"] = [np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_translation_error_mean"] = [np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_translation_error_std"] = [np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_rotation_error_mean"] = [np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_rotation_error_std"] = [np.nan]

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error(f"{self.metric_name}: ground_truth_poses file not found:\n{self.ground_truth_poses_file_path}")
            return False

        if not path.isfile(self.localization_update_poses_file_path):
            print_error(f"{self.metric_name}: estimated_localization_update_poses file not found:\n{self.localization_update_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]
        collision_time = self.results_df['collision_time'].iloc[0] if self.results_df['collision_rate'].iloc[0] == 1 else None

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t
        if collision_time is not None:
            navigation_end_time = min(navigation_end_time, collision_time)

        # get the dataframes for ground truth poses
        ground_truth_poses_df = pd.read_csv(self.ground_truth_poses_file_path)
        ground_truth_poses_df = ground_truth_poses_df[(navigation_start_time <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= navigation_end_time)]

        # get the dataframes for localization update poses
        localization_update_poses_df = pd.read_csv(self.localization_update_poses_file_path)
        if len(localization_update_poses_df) == 0:
            if self.verbose:
                if self.verbose:
                    print_info(f"{self.metric_name}: not enough localization update poses in navigation interval [{navigation_start_time}, {navigation_end_time}]:\n{self.localization_update_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True
        localization_update_poses_df = localization_update_poses_df[(navigation_start_time <= localization_update_poses_df.t) & (localization_update_poses_df.t <= navigation_end_time)]

        # compute the interpolated ground truth poses
        try:
            interpolated_df = interpolate_pose_2d_trajectories(
                trajectory_a_df=localization_update_poses_df, trajectory_a_label='est',
                trajectory_b_df=ground_truth_poses_df, trajectory_b_label='gt',
                limit_trajectory_a_rate=False, interpolation_tolerance=0.1
            )
        except InterpolationException as e:
            print_info(f"{self.metric_name}: interpolation exception: {e} when interpolating:\n{self.ground_truth_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        # if not enough matching ground truth data points are found, the metrics can not be computed
        if len(interpolated_df.index) < 2:
            print_info(f"{self.metric_name}: no matching ground truth data points were found when interpolating:\n{self.ground_truth_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        interpolated_poses = interpolated_df[['t', 'x_est', 'y_est', 'theta_est', 'x_gt', 'y_gt', 'theta_gt']].values

        absolute_translation_errors = list()
        absolute_rotation_errors = list()
        relative_translation_errors = list()
        relative_rotation_errors = list()
        normalized_relative_translation_errors = list()
        normalized_relative_rotation_errors = list()
        for i in range(1, len(interpolated_df)):
            start_timestamp = interpolated_poses[i - 1, 0]
            end_timestamp = interpolated_poses[i, 0]

            # compute ground truth and estimate transforms from start to end
            ground_truth_start_pose = interpolated_poses[i - 1, 4:7]
            x_gt, y_gt, theta_gt = ground_truth_end_pose = interpolated_poses[i, 4:7]
            est_start_pose = interpolated_poses[i - 1, 1:4]
            x_est, y_est, theta_est = est_end_pose = interpolated_poses[i, 1:4]

            # compute absolute error
            absolute_translation_error = np.sqrt((x_gt - x_est) ** 2 + (y_gt - y_est) ** 2)
            absolute_rotation_error = np.abs(np.arctan2(np.sin(theta_gt - theta_est), np.cos(theta_gt - theta_est)))

            # compute relative error
            relative_error_x, relative_error_y, relative_error_theta = relative_2d_pose_transform(ground_truth_start_pose, ground_truth_end_pose, est_start_pose, est_end_pose)
            relative_translation_error = np.sqrt(relative_error_x ** 2 + relative_error_y ** 2)
            relative_rotation_error = np.abs(np.arctan2(np.sin(relative_error_theta), np.cos(relative_error_theta)))

            # compute the integral of translation between updates
            ground_truth_poses_df_clipped = ground_truth_poses_df[(start_timestamp <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= end_timestamp)]
            ground_truth_positions = ground_truth_poses_df_clipped[['x', 'y']].values
            squared_deltas = (ground_truth_positions[1:-1] - ground_truth_positions[0:-2]) ** 2  # equivalent to (x_2-x_1)**2, (y_2-y_1)**2, for each row
            trajectory_translation = np.sqrt(np.sum(squared_deltas, axis=1)).sum()  # equivalent to sum for each row of sqrt( (x_2-x_1)**2 + (y_2-y_1)**2 )

            # compute the integral of rotation between updates
            ground_truth_rotations = ground_truth_poses_df_clipped['theta'].values
            rotation_differences = ground_truth_rotations[1:-1] - ground_truth_rotations[0:-2]
            rotation_deltas = np.abs(np.arctan2(np.sin(rotation_differences), np.cos(rotation_differences)))  # normalize the angle difference
            trajectory_rotation = np.sum(rotation_deltas)

            absolute_translation_errors.append(absolute_translation_error)
            absolute_rotation_errors.append(absolute_rotation_error)
            relative_translation_errors.append(relative_translation_error)
            relative_rotation_errors.append(relative_rotation_error)
            if trajectory_translation > 0.01:  # only compute this errors if there was a meaningful translation (1cm)
                normalized_relative_translation_errors.append(relative_translation_error / trajectory_translation)
            if trajectory_rotation > 0.01:  # only compute this errors if there was a meaningful rotation (0.01rad =~ 0.6deg)
                normalized_relative_rotation_errors.append(relative_rotation_error/trajectory_rotation)

        self.results_df["collisionless_localization_update_absolute_translation_error_mean"] = [float(np.mean(absolute_translation_errors)) if len(absolute_translation_errors) else np.nan]
        self.results_df["collisionless_localization_update_absolute_translation_error_std"] = [float(np.std(absolute_translation_errors)) if len(absolute_translation_errors) else np.nan]
        self.results_df["collisionless_localization_update_absolute_rotation_error_mean"] = [float(np.mean(absolute_rotation_errors)) if len(absolute_rotation_errors) else np.nan]
        self.results_df["collisionless_localization_update_absolute_rotation_error_std"] = [float(np.std(absolute_rotation_errors)) if len(absolute_rotation_errors) else np.nan]
        self.results_df["collisionless_localization_update_relative_translation_error_mean"] = [float(np.mean(relative_translation_errors)) if len(relative_translation_errors) else np.nan]
        self.results_df["collisionless_localization_update_relative_translation_error_std"] = [float(np.std(relative_translation_errors)) if len(relative_translation_errors) else np.nan]
        self.results_df["collisionless_localization_update_relative_rotation_error_mean"] = [float(np.mean(relative_rotation_errors)) if len(relative_rotation_errors) else np.nan]
        self.results_df["collisionless_localization_update_relative_rotation_error_std"] = [float(np.std(relative_rotation_errors)) if len(relative_rotation_errors) else np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_translation_error_mean"] = [float(np.mean(normalized_relative_translation_errors)) if len(normalized_relative_translation_errors) else np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_translation_error_std"] = [float(np.std(normalized_relative_translation_errors)) if len(normalized_relative_translation_errors) else np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_rotation_error_mean"] = [float(np.mean(normalized_relative_rotation_errors)) if len(normalized_relative_rotation_errors) else np.nan]
        self.results_df["collisionless_localization_update_normalized_relative_rotation_error_std"] = [float(np.std(normalized_relative_rotation_errors)) if len(normalized_relative_rotation_errors) else np.nan]

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class LocalizationUpdateRate:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.localization_update_poses_file_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "localization_update_rate"
        self.version = 1

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["localization_update_rate_mean"] = [np.nan]
        self.results_df["localization_update_rate_std"] = [np.nan]

        if not path.isfile(self.localization_update_poses_file_path):
            print_error(f"{self.metric_name}: estimated_localization_update_poses file not found:\n{self.localization_update_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for localization update poses
        localization_update_poses_df = pd.read_csv(self.localization_update_poses_file_path)
        if len(localization_update_poses_df) == 0:
            if self.verbose:
                print_info(f"{self.metric_name}: no localization update poses in:\n{self.localization_update_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True
        localization_update_timestamps = localization_update_poses_df[(navigation_start_time <= localization_update_poses_df.t) & (localization_update_poses_df.t <= navigation_end_time)]['t'].values

        if len(localization_update_timestamps) < 2:
            if self.verbose:
                print_info(f"{self.metric_name}: not enough localization update poses in navigation interval [{navigation_start_time}, {navigation_end_time}]:\n{self.localization_update_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        localization_update_deltas = localization_update_timestamps[1:] - localization_update_timestamps[0:-1]
        self.results_df["localization_update_rate_mean"] = [float(np.mean(localization_update_deltas))]
        self.results_df["localization_update_rate_std"] = [float(np.std(localization_update_deltas))]
        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class MotionCharacteristics:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "motion_characteristics"
        self.version = 4

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["average_translation_velocity"] = [np.nan]
        self.results_df["average_rotation_velocity"] = [np.nan]
        self.results_df["translation_rotation_product"] = [np.nan]
        self.results_df["average_translation_acceleration"] = [np.nan]
        self.results_df["average_rotation_acceleration"] = [np.nan]
        self.results_df["translation_rotation_acceleration_product"] = [np.nan]

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error(f"{self.metric_name}: ground_truth_poses file not found:\n{self.ground_truth_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for ground truth poses
        ground_truth_poses_df = pd.read_csv(self.ground_truth_poses_file_path)
        ground_truth_poses_df = ground_truth_poses_df[(navigation_start_time <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= navigation_end_time)]

        # compute derivative of velocity
        ground_truth_poses_df = ground_truth_poses_df.rolling(11, center=True).mean()
        ground_truth_poses_df['v_tran'] = np.sqrt(ground_truth_poses_df.v_x**2 + ground_truth_poses_df.v_y**2)
        ground_truth_poses_df['v_rot'] = ground_truth_poses_df.v_theta
        diff_df = ground_truth_poses_df.diff()
        diff_df['a_tran'] = np.sqrt((diff_df.v_x/diff_df.t)**2 + (diff_df.v_y/diff_df.t)**2)
        diff_df['a_rot'] = diff_df.v_theta/diff_df.t
        diff_df = diff_df.rolling(11, center=True).mean()

        vt = np.abs(ground_truth_poses_df.v_tran)
        vr = np.abs(ground_truth_poses_df.v_rot)
        at = np.abs(diff_df.a_tran)
        ar = np.abs(diff_df.a_rot)

        self.results_df["average_velocity_atan"] = [float(np.mean(np.arctan2(vr, vt)))]
        self.results_df["average_translation_velocity"] = [float(np.mean(vt))]
        self.results_df["average_rotation_velocity"] = [float(np.mean(vr))]
        self.results_df["translation_rotation_product"] = [float(np.mean(vt * vr))]
        self.results_df["average_translation_acceleration"] = [float(np.mean(at))]
        self.results_df["average_rotation_acceleration"] = [float(np.mean(ar))]
        self.results_df["translation_rotation_acceleration_product"] = [float(np.mean(at * ar))]
        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class CmdVel:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.cmd_vel_file_path = path.join(run_output_folder, "benchmark_data", "cmd_vel.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "cmd_vel_metrics"
        self.version = 2

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["max_cmd_vel_translation"] = [np.nan]
        self.results_df["max_cmd_vel_rotation"] = [np.nan]
        self.results_df["mean_cmd_vel_translation"] = [np.nan]
        self.results_df["mean_cmd_vel_rotation"] = [np.nan]

        # check required files exist
        if not path.isfile(self.cmd_vel_file_path):
            print_error(f"{self.metric_name}: cmd_vel file not found:\n{self.cmd_vel_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for ground truth poses
        cmd_vel_df = pd.read_csv(self.cmd_vel_file_path)
        cmd_vel_df = cmd_vel_df[(navigation_start_time <= cmd_vel_df.t) & (cmd_vel_df.t <= navigation_end_time)]

        if len(cmd_vel_df) == 0:
            self.results_df["max_cmd_vel_translation"] = [0.0]
            self.results_df["max_cmd_vel_rotation"] = [0.0]
            self.results_df["mean_cmd_vel_translation"] = [0.0]
            self.results_df["mean_cmd_vel_rotation"] = [0.0]
            self.results_df[f"{self.metric_name}_version"] = [self.version]
        else:
            translation_cmds = np.sqrt(cmd_vel_df.linear_x.values**2 + cmd_vel_df.linear_y.values**2)
            rotation_cmds = cmd_vel_df.angular_z

            self.results_df["max_cmd_vel_translation"] = [float(np.max(translation_cmds))]
            self.results_df["max_cmd_vel_rotation"] = [float(np.max(rotation_cmds))]
            self.results_df["mean_cmd_vel_translation"] = [float(np.mean(translation_cmds))]
            self.results_df["mean_cmd_vel_rotation"] = [float(np.mean(rotation_cmds))]
            self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class NormalizedCurvature:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "normalized_curvature"
        self.version = 1


    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error(f"{self.metric_name}: ground_truth_poses file not found:\n{self.ground_truth_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df[self.metric_name] = [np.nan]

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for ground truth poses
        ground_truth_poses_df = pd.read_csv(self.ground_truth_poses_file_path)

        ground_truth_poses_df_clipped = ground_truth_poses_df[(navigation_start_time <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= navigation_end_time)]
        ground_truth_positions = ground_truth_poses_df_clipped[['x', 'y']].values

        curvature = 0
        # number of (x,y) points in the table
        sample_size = len(ground_truth_positions)

        if(sample_size > 2):
            # if path has more than 2 points then consider a triplet of contiguous points (p1,p2,p3) and compute the internal angle adjacent to p2 
            for i in range(sample_size-2):
                p1 = ground_truth_positions[i]
                p2 = ground_truth_positions[i+1]
                p3 = ground_truth_positions[i+2]
                
                delta_x = (p2[0] - p1[0]) * (p3[0] - p2[0])
                delta_y = (p2[1] - p1[1]) * (p3[1] - p2[1])
                dist_p1_p2 = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                dist_p2_p3 = math.sqrt((p2[0] - p3[0])**2 + (p2[1] - p3[1])**2)
                angle = np.arccos((delta_x + delta_y) / (dist_p1_p2 * dist_p2_p3))
                # if angle is equal to pi then the path is straight, so it makes no sense to compute the curvature
                if (angle == math.pi):
                    continue
                curvature += angle
        
        normalized_curvature = curvature / sample_size

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df[self.metric_name] = [float(normalized_curvature)]
        return True


class PedestrianEncounters:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False, verbose=True):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.pedestrian_poses_file_path = path.join(run_output_folder, "benchmark_data", "pedestrian_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.run_info_path = path.join(run_output_folder, "run_info.yaml")
        self.recompute_anyway = recompute_anyway
        self.verbose = verbose
        self.metric_name = "pedestrian_encounters"
        self.version = 4
        self.th_high = 3.0 
        self.th_low = 1.5

    def compute(self):

        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["pedestrian_encounters"] = [np.nan]

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error(f"{self.metric_name}: ground_truth_poses file not found:\n{self.ground_truth_poses_file_path}")
            return False
        
        if not path.isfile(self.pedestrian_poses_file_path):
            print_error(f"{self.metric_name}: pedestrian_poses file not found:\n{self.pedestrian_poses_file_path}")
            return False
        
        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found:\n{self.run_events_file_path}")
            return False
        
        with open(self.run_info_path) as run_info_file:
            run_info = yaml.safe_load(run_info_file)
        num_pedestrians = run_info['run_parameters']['pedestrian_number']

        if num_pedestrians == 0:
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            self.results_df["pedestrian_encounters"] = [0]
            return True

        # get timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_info(f"{self.metric_name}: event navigation_goal_accepted not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_info(f"{self.metric_name}: events navigation_succeeded and navigation_failed not in events file:\n{self.run_events_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        navigation_start_time = navigation_start_events.iloc[0].t
        navigation_end_time = navigation_succeeded_events.iloc[0].t if len(navigation_succeeded_events) == 1 else navigation_failed_events.iloc[0].t

        # get the dataframes for ground truth poses
        ground_truth_poses_df = pd.read_csv(self.ground_truth_poses_file_path)
        ground_truth_poses_df = ground_truth_poses_df[(navigation_start_time <= ground_truth_poses_df.t) & (ground_truth_poses_df.t <= navigation_end_time)]

        # get the dataframes for pedestrian poses
        try:
            pedestrian_poses_df = pd.read_csv(self.pedestrian_poses_file_path)
        except pd.errors.EmptyDataError as e:
            print_error(f"{self.metric_name}: pedestrian_poses file is emty:\n{self.pedestrian_poses_file_path}")
            return False

        if len(pedestrian_poses_df) == 0:
            print_info(f"{self.metric_name}: not enough pedestrian poses in navigation interval [{navigation_start_time}, {navigation_end_time}]:\n{self.odometry_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        # variable holding number of encounters between robot and every pedestrian
        encounter_count = 0

        # compute the interpolated ground truth poses for each pedestrian
        for i in range(num_pedestrians):
            # build an interpolated dataframe in the form [t, x_robot, y_robot, x_ped, y_ped]
            pedestrian_i_df = pedestrian_poses_df[['t', f'x_{i}', f'y_{i}', f'theta_{i}']].copy()
            pedestrian_i_df.rename(columns={f'x_{i}': 'x', f'y_{i}': 'y', f'theta_{i}': 'theta'}, inplace=True)

            interpolated_df = interpolate_pose_2d_trajectories(
                trajectory_a_df=ground_truth_poses_df, trajectory_a_label='robot',
                trajectory_b_df=pedestrian_i_df, trajectory_b_label='ped',
                limit_trajectory_a_rate=False, interpolation_tolerance=0.1
            )

            # array holding all the distances between pedestrian_i and the robot
            distance_array = np.sqrt( ( interpolated_df['x_robot'] - interpolated_df['x_ped'])**2 + (interpolated_df['y_robot'] - interpolated_df['y_ped'])**2 ) 
            
            # boolean variable representing an encounter between pedestrian_i and robot
            is_encountering = False
            for dist in distance_array:
                # when the distance is below a predefined threshold_low and no other encounter was made before, then the pedestrian is in the range of the robot, hence an encounter is happening.
                if dist < self.th_low and not is_encountering:
                    is_encountering = True
                    encounter_count += 1
                # when the distance is higher than a predefined threshold_high and an encounter was made before, then the pedestrian is not anymore in range of the robot, hence stop considering it an encounter anymore.
                if not dist < self.th_high and is_encountering:
                    is_encountering = False

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df["pedestrian_encounters"] = [int(encounter_count)]
        return True


