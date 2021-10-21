#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob
import pickle
from collections import defaultdict
from os import path
import numpy as np
import pandas as pd

from performance_modelling_py.utils import print_error


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
            print_error(f"{self.metric_name}: ground_truth_poses file not found {self.ground_truth_poses_file_path}")
            return False

        if not path.isfile(self.run_events_file_path):
            print_error(f"{self.metric_name}: run_events file not found {self.run_events_file_path}")
            return False

        # get waypoint timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_error("event navigation_goal_accepted not in events file")
            return False

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_error("events navigation_succeeded and navigation_failed not in events file")
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
            print_error(f"{self.metric_name}: run_events file not found {self.run_events_file_path}")
            return False

        # get waypoint timestamps info from run events
        run_events_df = pd.read_csv(self.run_events_file_path, engine='python', sep=', ')
        navigation_start_events = run_events_df[run_events_df.event == 'navigation_goal_accepted']
        navigation_succeeded_events = run_events_df[(run_events_df.event == 'navigation_succeeded')]
        navigation_failed_events = run_events_df[(run_events_df.event == 'navigation_failed')]

        if len(navigation_start_events) != 1:
            print_error("event navigation_goal_accepted not in events file")
            return False

        if len(navigation_succeeded_events) + len(navigation_failed_events) != 1:
            print_error("events navigation_succeeded and navigation_failed not in events file")
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
            print_error(f"{self.metric_name}: run_events file not found {self.run_events_file_path}")
            return False

        # get waypoint timestamps info from run events
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
        self.version = 2

    def compute(self):
        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and \
                f"{self.metric_name}_version" in self.results_df and \
                self.results_df.iloc[0][f"{self.metric_name}_version"] == self.version:
            return True

        # check required files exist
        if not path.isdir(self.ps_snapshots_folder_path):
            print_error(f"{self.metric_name}: ps_snapshots directory not found {self.ps_snapshots_folder_path}")
            return False

        ps_snapshot_files_path = path.join(self.ps_snapshots_folder_path, "ps_*.pkl")
        ps_snapshot_paths_list = sorted(glob.glob(ps_snapshot_files_path))
        if len(ps_snapshot_paths_list) == 0:
            print_error(f"{self.metric_name}: ps_snapshot files not found {ps_snapshot_files_path}")
            return False

        cpu_time_dict = defaultdict(int)
        max_memory_dict = defaultdict(int)

        for ps_snapshot_path in ps_snapshot_paths_list:
            try:
                with open(ps_snapshot_path, 'rb') as ps_snapshot_file:
                    ps_snapshot = pickle.load(ps_snapshot_file)
            except (EOFError, pickle.UnpicklingError) as e:
                print_error(f"{self.metric_name}: Could not load pickled ps snapshot. Error: {type(e)} {e}. Pickle file: {ps_snapshot_path}")
                continue
            for process_info in ps_snapshot:
                process_name = process_info['name']
                if process_name not in ['gzserver', 'gzclient', 'rviz2', 'local_planning_', ]:  # ignore simulator and rviz to count the robot system memory
                    cpu_time_dict[process_name] = max(
                        cpu_time_dict[process_name],
                        process_info['cpu_times'].user + process_info['cpu_times'].system
                    )
                    max_memory_dict[process_name] = max(
                        max_memory_dict[process_name],
                        process_info['memory_full_info'].pss
                    )

        if len(cpu_time_dict) == 0 or len(max_memory_dict) == 0:
            print_error(f"{self.metric_name}: no data from ps snapshots {ps_snapshot_files_path}")
            return False

        self.results_df["controller_cpu_time"] = [cpu_time_dict["controller_server"]]
        self.results_df["planner_cpu_time"] = [cpu_time_dict["planner_server"]]
        self.results_df["system_cpu_time"] = [sum(cpu_time_dict.values())]

        self.results_df["controller_max_memory"] = [max_memory_dict["controller_server"]]
        self.results_df["planner_max_memory"] = [max_memory_dict["planner_server"]]
        self.results_df["system_max_memory"] = [sum(max_memory_dict.values())]

        self.results_df[f"{self.metric_name}_version"] = [self.version]
        return True
