#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob
import argparse
from os import path

from local_planning_performance_modelling.local_planning_benchmark_run import BenchmarkRun
from performance_modelling_py.benchmark_execution.grid_benchmarking import execute_grid_benchmark


def main():

    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description='Execute the benchmark')
    default_environment_dataset_folders = "~/ds/performance_modelling/test_datasets/dataset/*"
    parser.add_argument('-e', dest='environment_dataset_folders',
                        help=f'Dataset folders containing the environment data. Use wildcards to select multiple folders. Only folders are selected, files are ignored. Defaults to {default_environment_dataset_folders}',
                        type=str,
                        default=default_environment_dataset_folders,
                        required=False)

    default_grid_benchmark_configuration = "grid_benchmark_all.yaml"
    parser.add_argument('-c', dest='grid_benchmark_configuration',
                        help=f'Yaml file with the configuration of the benchmark relative to the benchmark_configurations folder in this package. Defaults to {default_grid_benchmark_configuration}',
                        type=str,
                        default=default_grid_benchmark_configuration,
                        required=False)

    default_base_run_folder = "~/ds/performance_modelling/output/test_local_planning/"
    parser.add_argument('-r', dest='base_run_folder',
                        help=f'Folder in which the result of each run will be placed. Defaults to {default_base_run_folder}',
                        type=str,
                        default=default_base_run_folder,
                        required=False)

    default_num_runs = 1
    parser.add_argument('-n', '--num-runs', dest='num_runs',
                        help=f'Number of runs to be executed for each combination of configurations. Defaults to {default_num_runs}',
                        type=int,
                        default=default_num_runs,
                        required=False)

    parser.add_argument('--ignore-previous-runs', dest='ignore_previous_runs',
                        help='When set the the previous runs in base_run_folder are ignored when counting the param combinations already executed.',
                        action='store_true',
                        required=False)

    parser.add_argument('--no-shuffle', dest='no_shuffle',
                        help='When set the order of the combinations is not randomized.',
                        action='store_true',
                        required=False)

    parser.add_argument('--gui', dest='gui',
                        help='When set the components are run with GUI (opposite of headless).',
                        action='store_true',
                        required=False)

    parser.add_argument('-s', '--show-ros-info', dest='show_ros_info',
                        help='When set the component nodes are launched with output="screen".',
                        action='store_true',
                        required=False)

    args = parser.parse_args()
    base_run_folder = path.expanduser(args.base_run_folder)
    environment_folders = sorted(filter(path.isdir, glob.glob(path.expanduser(args.environment_dataset_folders))))
    grid_benchmark_configuration = path.join(path.expanduser("~/w/ros2_ws/src/local_planning_performance_modelling/config/benchmark_configurations/"), args.grid_benchmark_configuration)

    execute_grid_benchmark(benchmark_run_object=BenchmarkRun,
                           grid_benchmark_configuration=grid_benchmark_configuration,
                           environment_folders=environment_folders,
                           base_run_folder=base_run_folder,
                           num_runs=args.num_runs,
                           ignore_executed_params_combinations=args.ignore_previous_runs,
                           shuffle=not args.no_shuffle,
                           headless=not args.gui,
                           show_ros_info=args.show_ros_info)

