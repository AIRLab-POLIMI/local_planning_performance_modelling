#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob
import argparse
import os
import sys
from os import path

from local_planning_performance_modelling.local_planning_benchmark_run import BenchmarkRun
from performance_modelling_py.benchmark_execution.grid_benchmarking import execute_grid_benchmark
from performance_modelling_py.utils import print_error


def main():

    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description='Execute the benchmark')
    default_environment_dataset_folders = "~/ds/performance_modelling/test_datasets/dataset/*"
    parser.add_argument('-e', dest='environment_dataset_folders',
                        help=f'Dataset folders containing the environment data. Use wildcards to select multiple folders. Only folders are selected, files are ignored. Defaults to {default_environment_dataset_folders}',
                        type=str,
                        default=default_environment_dataset_folders,
                        required=False)

    benchmark_configurations_dir_path = path.expanduser("~/w/ros2_ws/src/local_planning_performance_modelling/config/benchmark_configurations/")
    default_benchmark_configuration_from_package = "grid_benchmark_all.yaml"
    parser.add_argument('-p', dest='benchmark_configuration_from_package',
                        help=f'Yaml file with the configuration of the benchmark relative to the config/benchmark_configurations folder in this package. If set, the option "benchmark_configuration" will be ignored.'
                             f' Defaults to {default_benchmark_configuration_from_package}. Available configuration files:\n' +
                             '\n'.join(sorted(os.listdir(benchmark_configurations_dir_path))),
                        type=str,
                        default=None,
                        required=False)

    default_benchmark_configuration = path.join(benchmark_configurations_dir_path, default_benchmark_configuration_from_package)
    parser.add_argument('-c', dest='benchmark_configuration',
                        help=f'Yaml file with the configuration of the benchmark. This option is ignored if the option benchmark_configuration_from_package is set. Defaults to {default_benchmark_configuration}.',
                        type=str,
                        default=default_benchmark_configuration,
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

    parser.add_argument('--rviz', dest='rviz',
                        help='When set rviz is launched.',
                        action='store_true',
                        required=False)

    parser.add_argument('--gzclient', dest='gzclient',
                        help='When set gzclient is launched.',
                        action='store_true',
                        required=False)

    parser.add_argument('--do-not-record', dest='do_not_record',
                        help='When set the rosbag recording is not launched.',
                        action='store_true',
                        required=False)

    args = parser.parse_args()
    base_run_folder = path.expanduser(args.base_run_folder)
    environment_folders = sorted(filter(path.isdir, glob.glob(path.expanduser(args.environment_dataset_folders))))
    if args.benchmark_configuration_from_package is not None:
        grid_benchmark_configuration = path.join(benchmark_configurations_dir_path, args.benchmark_configuration_from_package)
    else:
        grid_benchmark_configuration = path.expanduser(args.benchmark_configuration)
    if not path.exists(grid_benchmark_configuration):
        print_error(f"benchmark configuration file [{path.abspath(grid_benchmark_configuration)}] does not exists")
        sys.exit(-1)

    execute_grid_benchmark(benchmark_run_object=BenchmarkRun,
                           grid_benchmark_configuration=grid_benchmark_configuration,
                           environment_folders=environment_folders,
                           base_run_folder=base_run_folder,
                           num_runs=args.num_runs,
                           ignore_executed_params_combinations=args.ignore_previous_runs,
                           shuffle=not args.no_shuffle,
                           args_parser=args)
