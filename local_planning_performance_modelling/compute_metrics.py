#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import glob
import multiprocessing
import os
import sys
import traceback
from os import path

import numpy as np
import yaml
from yaml.constructor import ConstructorError

from performance_modelling_py.utils import print_info, print_error

import pandas as pd


class TrajectoryLength:
    def __init__(self, results_df, run_output_folder, recompute_anyway=False):
        self.results_df = results_df
        self.ground_truth_poses_file_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")
        self.run_events_file_path = path.join(run_output_folder, "benchmark_data", "run_events.csv")
        self.recompute_anyway = recompute_anyway
        self.version = 3

    def compute(self):

        # check required files exist
        if not path.isfile(self.ground_truth_poses_file_path):
            print_error("trajectory_length: ground_truth_poses file not found {}".format(self.ground_truth_poses_file_path))
            return False

        if not path.isfile(self.run_events_file_path):
            print_error("trajectory_length: run_events file not found {}".format(self.run_events_file_path))
            return False

        # Do not recompute the metric if it was already computed with the same version
        if not self.recompute_anyway and 'trajectory_length' in self.results_df and 'trajectory_length_version' in self.results_df and \
                self.results_df.iloc[0].trajectory_length_version == self.version:
            return True

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

        self.results_df['trajectory_length_version'] = [self.version]
        self.results_df['trajectory_length'] = [float(trajectory_length)]
        return True


def compute_run_metrics(run_output_folder, recompute_all_metrics=False):
    run_id = path.basename(run_output_folder)
    run_info_path = path.join(run_output_folder, "run_info.yaml")
    if not path.exists(run_info_path) or not path.isfile(run_info_path):
        print_error("run info file does not exists")
        return

    try:
        with open(run_info_path) as run_info_file:
            run_info = yaml.safe_load(run_info_file)
    except ConstructorError:
        print_error("Could not parse run_info.yaml for run {}".format(run_id))
        return

    # output files
    metrics_result_folder_path = path.join(run_output_folder, "metric_results")
    metrics_result_file_path = path.join(metrics_result_folder_path, "metrics.csv")

    if not path.exists(metrics_result_folder_path):
        os.makedirs(metrics_result_folder_path)

    if path.exists(metrics_result_file_path):
        results_df = pd.read_csv(metrics_result_file_path)
    else:
        results_df = pd.DataFrame()

    # trajectory_length
    TrajectoryLength(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()

    # # cpu_and_memory_usage
    # if recompute_all_metrics or 'cpu_and_memory_usage' not in metrics_result_dict:
    #     print_info("cpu_and_memory_usage {}".format(run_id))
    #     ps_snapshots_folder_path = path.join(run_output_folder, "benchmark_data", "ps_snapshots")
    #     metrics_result_dict['cpu_and_memory_usage'] = cpu_and_memory_usage_metrics(ps_snapshots_folder_path)

    # write metrics
    # with open(metrics_result_file_path, 'w') as metrics_result_file:
    #     yaml.dump(metrics_result_dict, metrics_result_file, default_flow_style=False)
    results_df.to_csv(metrics_result_file_path, index=False)


def parallel_compute_metrics(run_output_folder, recompute_all_metrics):
    print_info("start : compute_metrics {:3d}% {}".format(int((shared_progress.value + 1) * 100 / shared_num_runs.value), path.basename(run_output_folder)))

    # noinspection PyBroadException
    try:
        compute_run_metrics(run_output_folder, recompute_all_metrics=recompute_all_metrics)
    except KeyboardInterrupt:
        print_info("parallel_compute_metrics: metrics computation interrupted (run {})".format(run_output_folder))
        sys.exit()
    except:
        print_error("parallel_compute_metrics: failed metrics computation for run {}".format(run_output_folder))
        print_error(traceback.format_exc())

    shared_progress.value += 1
    print_info("finish: compute_metrics {:3d}% {}".format(int(shared_progress.value * 100 / shared_num_runs.value), path.basename(run_output_folder)))


def main():
    print_info("Python version:", sys.version_info)
    default_base_run_folder = "~/ds/performance_modelling/output/test_local_planning/*"
    default_num_parallel_threads = 4
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description='Compute metrics for run directories in parallel.')

    parser.add_argument('--recompute', dest='recompute_all_metrics',
                        help='Whether to recompute all metrics. Defaults is false.'.format(default_base_run_folder),
                        action='store_true',
                        required=False)

    parser.add_argument('-r', dest='base_run_folder',
                        help='Folder in which the result of each run will be placed. Defaults to {}.'.format(default_base_run_folder),
                        type=str,
                        default=default_base_run_folder,
                        required=False)

    parser.add_argument('-j', dest='num_parallel_threads',
                        help='Number of parallel threads. Defaults to {}.'.format(default_num_parallel_threads),
                        type=int,
                        default=default_num_parallel_threads,
                        required=False)

    args = parser.parse_args()

    def is_completed_run_folder(p):
        return path.isdir(p) and path.exists(path.join(p, "RUN_COMPLETED"))

    def is_not_completed_run_folder(p):
        return path.isdir(p) and not path.exists(path.join(p, "RUN_COMPLETED"))

    run_folders = list(filter(is_completed_run_folder, glob.glob(path.expanduser(args.base_run_folder))))
    not_completed_run_folders = list(filter(is_not_completed_run_folder, glob.glob(path.expanduser(args.base_run_folder))))
    num_runs = len(run_folders)

    if len(not_completed_run_folders):
        print_info("Runs not completed:")
        for not_completed_run_folder in not_completed_run_folders:
            print(not_completed_run_folder)

    if len(run_folders) == 0:
        print_info("Nothing to do.")
        sys.exit(0)

    global shared_progress
    shared_progress = multiprocessing.Value('i', 0)
    global shared_num_runs
    shared_num_runs = multiprocessing.Value('i', len(run_folders))
    with multiprocessing.Pool(processes=args.num_parallel_threads) as pool:
        pool.starmap(parallel_compute_metrics, zip(run_folders, [args.recompute_all_metrics] * num_runs))


shared_progress = None
shared_num_runs = None
if __name__ == '__main__':
    main()
