#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob
import pickle
from collections import defaultdict
from os import path
import numpy as np
import pandas as pd

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
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
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

        squared_deltas = (ground_truth_positions[1:-1] - ground_truth_positions[0:-2]) ** 2  # equivalent to (x_2-x_1)**2, (y_2-y_1)**2, for each row
        sum_of_squared_deltas = np.sum(squared_deltas, axis=1)  # equivalent to (x_2-x_1)**2 + (y_2-y_1)**2, for each row
        euclidean_distance_of_deltas = np.sqrt(sum_of_squared_deltas)  # equivalent to sqrt( (x_2-x_1)**2 + (y_2-y_1)**2 ), for each row
        trajectory_length = euclidean_distance_of_deltas.sum()

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df[self.metric_name] = [float(trajectory_length)]
        return True


class ExecutionTime:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
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
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
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

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        self.results_df[self.metric_name] = [int(navigation_goal_reached)]
        return True


class CpuTimeAndMaxMemoryUsage:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.ps_snapshots_folder_path = path.join(run_output_folder, "benchmark_data", "ps_snapshots")
        self.recompute_anyway = recompute_anyway
        self.metric_name = "cpu_time_and_max_memory"
        self.version = 1

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # clear fields in case the computation fails so that the old data (from a previous version) will be removed
        self.results_df["controller_cpu_time"] = [np.nan]
        self.results_df["planner_cpu_time"] = [np.nan]
        self.results_df["system_cpu_time"] = [np.nan]
        self.results_df["controller_max_memory"] = [np.nan]
        self.results_df["planner_max_memory"] = [np.nan]
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

        cpu_time_dict = defaultdict(int)
        max_memory_dict = defaultdict(int)

        for ps_snapshot_path in ps_snapshot_paths_list:
            try:
                with open(ps_snapshot_path, 'rb') as ps_snapshot_file:
                    ps_snapshot = pickle.load(ps_snapshot_file)
            except (EOFError, pickle.UnpicklingError) as e:
                print_error(f"{self.metric_name}: Could not load pickled ps snapshot. Error: {type(e)} {e}. Pickle file:\n{ps_snapshot_path}")
                continue
            for process_info in ps_snapshot:
                process_name = process_info['name']
                if process_name not in ['gzserver', 'gzclient', 'rviz2', 'local_planning_']:  # ignore simulator and rviz to count the robot system memory
                    cpu_time_dict[process_name] = max(
                        cpu_time_dict[process_name],
                        process_info['cpu_times'].user + process_info['cpu_times'].system
                    )
                    max_memory_dict[process_name] = max(
                        max_memory_dict[process_name],
                        process_info['memory_full_info'].pss
                    )

        if len(cpu_time_dict) == 0 or len(max_memory_dict) == 0:
            print_error(f"{self.metric_name}: no data from ps snapshots:\n{ps_snapshot_files_path}")
            return False

        self.results_df["controller_cpu_time"] = [cpu_time_dict["controller_server"]]
        self.results_df["planner_cpu_time"] = [cpu_time_dict["planner_server"]]
        self.results_df["system_cpu_time"] = [sum(cpu_time_dict.values())]

        self.results_df["controller_max_memory"] = [max_memory_dict["controller_server"]]
        self.results_df["planner_max_memory"] = [max_memory_dict["planner_server"]]
        self.results_df["system_max_memory"] = [sum(max_memory_dict.values())]

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True


class OdometryError:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.odometry_poses_file_path = path.join(run_output_folder, "benchmark_data", "odom.csv")
        self.localization_update_poses_file_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.metric_name = "odometry_error"
        self.version = 1

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
            if len(interpolated_df_clipped) == 0:
                print_info(f"{self.metric_name}: ground truth rate too low compared to localization update rate in update timestamps interval [{start_timestamp}, {end_timestamp}]:\n{self.ground_truth_poses_file_path}")
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
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.localization_update_poses_file_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.metric_name = "localization_update_error"
        self.version = 1

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


class LocalizationUpdateRate:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.localization_update_poses_file_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
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
            print_info(f"{self.metric_name}: no localization update poses in:\n{self.localization_update_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True
        localization_update_timestamps = localization_update_poses_df[(navigation_start_time <= localization_update_poses_df.t) & (localization_update_poses_df.t <= navigation_end_time)]['t'].values

        if len(localization_update_timestamps) < 2:
            print_info(f"{self.metric_name}: not enough localization update poses in navigation interval [{navigation_start_time}, {navigation_end_time}]:\n{self.localization_update_poses_file_path}")
            self.results_df[f"{self.metric_name}_version"] = [self.version]
            return True

        localization_update_deltas = localization_update_timestamps[1:] - localization_update_timestamps[0:-1]
        self.results_df["localization_update_rate_mean"] = [float(np.mean(localization_update_deltas))]
        self.results_df["localization_update_rate_std"] = [float(np.std(localization_update_deltas))]
        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True
