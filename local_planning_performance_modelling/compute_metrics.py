#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import glob
import multiprocessing
import os
import sys
import traceback
from datetime import datetime
from functools import reduce
from os import path
import pandas as pd
import yaml
from yaml.constructor import ConstructorError

from performance_modelling_py.utils import print_info, print_error
from local_planning_performance_modelling.metrics import CpuTimeAndMaxMemoryUsage, TrajectoryLength, ExecutionTime, SuccessRate, OdometryError, LocalizationError, LocalizationUpdateRate


def compute_run_metrics(run_output_folder, recompute_all_metrics=False):

    # open the existing metrics file or make a new data frame
    metrics_result_folder_path = path.join(run_output_folder, "metric_results")
    if not path.exists(metrics_result_folder_path):
        os.makedirs(metrics_result_folder_path)
    metrics_result_file_path = path.join(metrics_result_folder_path, "metrics.csv")
    if not recompute_all_metrics and path.exists(metrics_result_file_path):
        results_df = pd.read_csv(metrics_result_file_path)
    else:
        results_df = pd.DataFrame()

    # add run parameters to the results dataframe
    run_info_path = path.join(run_output_folder, "run_info.yaml")
    try:
        with open(run_info_path) as run_info_file:
            run_info = yaml.safe_load(run_info_file)
    except ConstructorError:
        print_error(f"Could not parse run_info:\n{run_info_file}")
        return
    run_id = path.basename(run_info['run_folder'])
    results_df['run_id'] = [run_id]
    for run_parameter, run_parameter_value in run_info['run_parameters'].items():
        results_df[run_parameter] = [run_parameter_value]

    # compute metrics
    CpuTimeAndMaxMemoryUsage(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()
    TrajectoryLength(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()
    ExecutionTime(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()
    SuccessRate(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()
    OdometryError(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()
    LocalizationError(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()
    LocalizationUpdateRate(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics).compute()

    # write the metrics data frame to file
    results_df.to_csv(metrics_result_file_path, index=False)
    return results_df, list(run_info['run_parameters'].keys())


def parallel_compute_metrics(run_output_folder, recompute_all_metrics):
    print(f"start : compute_metrics {int((shared_progress.value + 1) * 100 / shared_num_runs.value):3d}% {path.basename(run_output_folder)}")

    results_df = None
    run_parameter_names = None
    # noinspection PyBroadException
    try:
        results_df, run_parameter_names = compute_run_metrics(run_output_folder, recompute_all_metrics=recompute_all_metrics)
    except KeyboardInterrupt:
        print_info(f"parallel_compute_metrics: metrics computation interrupted (run {run_output_folder})")
        sys.exit()
    except:
        print_error(f"parallel_compute_metrics: failed metrics computation for run {run_output_folder}")
        print_error(traceback.format_exc())

    shared_progress.value += 1
    print(f"finish: compute_metrics {int(shared_progress.value * 100 / shared_num_runs.value):3d}% {path.basename(run_output_folder)}")
    return results_df, run_parameter_names


def main():
    default_base_run_folder = "~/ds/performance_modelling/output/test_local_planning/*"
    default_num_parallel_threads = 16
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description='Compute metrics for run directories in parallel.')

    parser.add_argument('--recompute', dest='recompute_all_metrics',
                        help='Whether to recompute all metrics. Defaults is false.',
                        action='store_true',
                        required=False)

    parser.add_argument('-r', dest='base_run_folder',
                        help=f'Folder in which the result of each run will be placed. Defaults to {default_base_run_folder}.',
                        type=str,
                        default=default_base_run_folder,
                        required=False)

    parser.add_argument('-o', dest='output_dir_path',
                        help='Path of the output directory in which is written the dataframe (csv) with the results of all runs. Defaults to the directory of base_run_folder (where the run folders are)',
                        type=str,
                        default=None,
                        required=False)

    parser.add_argument('-j', dest='num_parallel_threads',
                        help='Number of parallel threads. Defaults to {default_num_parallel_threads}.',
                        type=int,
                        default=default_num_parallel_threads,
                        required=False)

    args = parser.parse_args()

    if args.output_dir_path is None:
        output_dir_path = path.dirname(path.expanduser(args.base_run_folder))
    else:
        output_dir_path = path.join(path.expanduser(args.output_path))
    if not path.exists(output_dir_path):
        os.makedirs(output_dir_path)

    results_path = path.join(output_dir_path, "results.csv")
    results_info_path = path.join(output_dir_path, "results_info.yaml")

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
        try:
            results_dfs_and_parameter_names = pool.starmap(parallel_compute_metrics, zip(run_folders, [args.recompute_all_metrics] * num_runs))
        except KeyboardInterrupt:
            print_info("main: metrics computation interrupted")
            sys.exit()

        results_dfs, run_parameter_names_lists = zip(*results_dfs_and_parameter_names)
        print("done.")

        all_results_df = pd.concat(filter(lambda d: d is not None, results_dfs), sort=False)
        all_results_df.to_csv(results_path, index=False)
        run_parameter_names = list(set(reduce(list.__add__, run_parameter_names_lists)))

        results_info = {'run_parameter_names': run_parameter_names, 'datetime': datetime.now().astimezone().isoformat()}
        with open(results_info_path, 'w') as results_info_file:
            yaml.dump(results_info, results_info_file)


shared_progress = multiprocessing.Value('i', 0)
shared_num_runs = multiprocessing.Value('i', 0)
if __name__ == '__main__':
    main()
