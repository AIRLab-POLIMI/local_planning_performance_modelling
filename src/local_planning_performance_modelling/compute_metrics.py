#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import glob
import multiprocessing
import os
import subprocess
import sys
import traceback
from datetime import datetime
from functools import reduce
from os import path
import pandas as pd
import yaml
from yaml.constructor import ConstructorError

from performance_modelling_py.utils import print_info, print_error
from local_planning_performance_modelling.metrics import CpuTimeAndMaxMemoryUsage, TrajectoryLength, ExecutionTime, SuccessRate, OdometryError, LocalizationError, CollisionlessLocalizationError, LocalizationUpdateRate, CollisionRate, Clearance, MotionCharacteristics, CmdVel, NormalizedCurvature, PedestrianEncounters, RealTimeFactor


def du(p):
    """disk usage in human readable format (e.g. '2,1GB')"""
    if path.exists(p):
        return subprocess.check_output(['du', '-shx', p]).split()[0].decode('utf-8')
    else:
        return "---"


def compress_dir(dir_path):
    working_dir = path.dirname(dir_path)
    compressed_file_filename = path.basename(dir_path)
    compressed_file_filename_w_ext = f'{compressed_file_filename}.tar.xz'
    compressed_file_path = path.join(working_dir, compressed_file_filename_w_ext)

    # if the compressed file already exists, do nothing (rather than overwriting it)
    if path.exists(compressed_file_path):
        if compression_debug_info:
            print_info(f"compress_dir: the compressed file already exists. Doing nothing.")
        return

    # if the directory does not exist, do nothing. This should not happen, because we already checked that the compressed file does not exist.
    if not path.exists(dir_path):
        print_error(f"compress_dir: neither the target directory nor the compressed file exist. DATA WAS LOST. dir_path: {dir_path}")
        return

    original_size = du(dir_path)
    compress_cmd = ['tar', '-cJf', compressed_file_filename_w_ext, compressed_file_filename]

    if compression_debug_info:
        print_info(f"compress_dir: compressing {dir_path}: {' '.join(compress_cmd)} in {working_dir}")

    subprocess.call(compress_cmd, cwd=working_dir)  # TODO and check that the compressed file is ok and that all files were compressed (for example because of an interrupt received during compression, which is likely)

    if compression_debug_info:
        print_info(f"compress_dir: {original_size} -> {du(compressed_file_path)}")


def decompress_dir(target_dir_path):

    compressed_file_path = f"{target_dir_path}.tar.xz"

    # if the target dir already exists, do nothing (rather than overwriting it)
    if path.exists(target_dir_path):
        if compression_debug_info:
            print_info(f"decompress_dir: the target dir already exists. Doing nothing.")
        return

    # if the compressed file does not exist, do nothing. This should not happen, because we already checked that the target dir does not exist.
    if not path.exists(compressed_file_path):  # TODO and check that the compressed file is ok
        print_error(f"decompress_dir: neither the target directory nor the compressed file exist. DATA WAS LOST. target_dir_path: {target_dir_path}")
        return

    working_dir = path.dirname(target_dir_path)
    decompressed_dir_path = path.basename(target_dir_path)

    original_size = du(compressed_file_path)
    decompress_cmd = ['tar', '-xJf', compressed_file_path]
    if compression_debug_info:
        print_info(f"decompress_dir: decompressing {target_dir_path}: {' '.join(decompress_cmd)} in {working_dir}")

    subprocess.call(decompress_cmd, cwd=working_dir)

    if compression_debug_info:
        print_info(f"decompress_dir: {original_size} -> {du(path.join(working_dir, decompressed_dir_path))}")


def remove_non_compressed_dir(non_compressed_dir_path):

    working_dir = path.dirname(non_compressed_dir_path)
    compressed_file_filename = path.basename(non_compressed_dir_path)
    compressed_file_filename_w_ext = f'{compressed_file_filename}.tar.xz'
    compressed_file_path = path.join(working_dir, compressed_file_filename_w_ext)

    # check if the non compressed directory (to be removed) already doesn't exists.
    if not path.exists(non_compressed_dir_path):
        # if the non compressed directory (to be removed) already doesn't exists, but the compress data exists, then it is ok.
        # this happens when the data is not decompressed, because no metric needed to be computed.
        if path.exists(compressed_file_path):  # TODO and check that the compressed file is ok
            if compression_debug_info:
                print_info(f"remove_non_compressed_dir: attempting to remove non compressed data, but non compressed data does not exists. It is ok because the compressed data exists.")
        # if neither non compressed directory (to be removed) nor the compressed file exist, then there was a loss of data.
        else:
            print_error(f"remove_non_compressed_dir: neither the non compressed directory nor the compressed file exist. DATA WAS LOST. non_compressed_dir_path: {non_compressed_dir_path}")
        return

    # check loss of data
    if not path.exists(compressed_file_path):  # TODO and check that the compressed file is ok
        print_error(f"remove_non_compressed_dir: attempting to remove non compressed data, but compressed data does not exists. PREVENTING LOSS OF DATA! Missing compressed file path: {compressed_file_path}")
        return

    rm_cmd = ['trash', non_compressed_dir_path]

    if compression_debug_info:
        print_info(f"removing {non_compressed_dir_path}: {' '.join(rm_cmd)}")

    subprocess.call(rm_cmd)


def remove_compressed_file(non_compressed_dir_path):

    working_dir = path.dirname(non_compressed_dir_path)
    compressed_file_filename = path.basename(non_compressed_dir_path)
    compressed_file_filename_w_ext = f'{compressed_file_filename}.tar.xz'
    compressed_file_path = path.join(working_dir, compressed_file_filename_w_ext)

    # check if the compressed file (to be removed) already doesn't exists.
    if not path.exists(non_compressed_dir_path):
        # this happens if the request is to not compress the data (which is why we are trying to delete the compressed file) and,
        # the metrics were not computed so the data was not decompressed (which is why the non compressed directory does not exists).
        if path.exists(compressed_file_path):  # TODO and check that the compressed file is ok
            if compression_debug_info:
                print_info(f"remove_compressed_file: attempting to remove compressed data, but non compressed data does not exists. non_compressed_dir_path: {non_compressed_dir_path}")
        # if neither exist, then there was loss of data
        else:
            print_error(f"remove_compressed_file: neither the non compressed directory nor the compressed file exist. DATA WAS LOST. non_compressed_dir_path: {non_compressed_dir_path}")
        return

    # check that the compressed file (to be removed) exists
    # the compressed file (to be removed) may not exist.
    if not path.exists(compressed_file_path):
        if compression_debug_info:
            print_info(f"remove_compressed_file: attempting to remove compressed data, and compressed data already does not exist. Doing nothing: {non_compressed_dir_path}")
        return

    rm_cmd = ['trash', compressed_file_path]

    if compression_debug_info:
        print_info(f"removing {compressed_file_path}: {' '.join(rm_cmd)}")

    subprocess.call(rm_cmd)


def compress_run_data(run_folder):
    compress_dir(path.join(run_folder, "benchmark_data"))


def decompress_run_data(run_folder):
    decompress_dir(path.join(run_folder, "benchmark_data"))


def remove_compressed_data(run_folder):
    remove_compressed_file(path.join(run_folder, "benchmark_data"))


def remove_non_compressed_data(run_folder):
    remove_non_compressed_dir(path.join(run_folder, "benchmark_data"))


class Timer:
    def __init__(self, name):
        if not print_timings:
            return
        self.name = name
        self.start_time = datetime.now()

    def print_timing(self):
        if print_timings:
            print(f"{self.name:<70} {'':<20}  {datetime.now() - self.start_time}")

    def print_not_computed(self):
        if print_timings:
            print(f"{self.name:<70} {'already computed':<20} ({datetime.now() - self.start_time})")


def compute_run_metrics(run_output_folder):

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
    metrics_to_compute = [
        CollisionRate(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        Clearance(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        TrajectoryLength(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        ExecutionTime(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        SuccessRate(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        OdometryError(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        LocalizationError(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        CollisionlessLocalizationError(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        LocalizationUpdateRate(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        MotionCharacteristics(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        # CpuTimeAndMaxMemoryUsage(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        CmdVel(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        NormalizedCurvature(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        # PedestrianEncounters(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent),
        RealTimeFactor(results_df=results_df, run_output_folder=run_output_folder, recompute_anyway=recompute_all_metrics, verbose=not silent)
    ]

    any_metric_needs_computation = False
    for m in metrics_to_compute:
        if m.needs_to_be_computed():
            any_metric_needs_computation = True

    if any_metric_needs_computation:
        decompress_run_data(run_output_folder)

    success = True
    for m in metrics_to_compute:
        timer = Timer(m.metric_name)
        if m.needs_to_be_computed():
            if not m.compute():
                success = False
                print_error(f"compute_run_metrics: failed metrics computation for run {run_output_folder}")
                break
            timer.print_timing()
        else:
            timer.print_not_computed()

    # write the metrics data frame to file
    results_df.to_csv(metrics_result_file_path, index=False)

    # compress the benchmark data if requested
    if compress_benchmark_data:
        compress_run_data(run_output_folder)
        remove_non_compressed_data(run_output_folder)

    # if data should not be compressed, remove the compressed data that may be left from previous compression requests
    if not compress_benchmark_data:
        remove_compressed_data(run_output_folder)

    return (results_df, list(run_info['run_parameters'].keys())) if success else (None, None)


def parallel_compute_metrics(run_output_folder):
    if not silent:
        print(f"start : compute_metrics {int((shared_progress.value + 1) * 100 / shared_num_runs.value):3d}% {path.basename(run_output_folder)}")

    results_df = None
    run_parameter_names = None
    # noinspection PyBroadException
    try:
        results_df, run_parameter_names = compute_run_metrics(run_output_folder)
    except KeyboardInterrupt:
        print_info(f"parallel_compute_metrics: metrics computation interrupted (run {run_output_folder})")
        sys.exit()
    except:
        print_error(f"parallel_compute_metrics: failed metrics computation for run {run_output_folder}")
        print_error(traceback.format_exc())

    shared_progress.value += 1
    if not silent:
        print(f"finish: compute_metrics {int(shared_progress.value * 100 / shared_num_runs.value):3d}% {path.basename(run_output_folder)}")
    return results_df, run_parameter_names, run_output_folder


def main():
    default_base_run_folder = "~/ds/performance_modelling/output/local_planning/*"
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
    
    default_alternative_base_run_folder = path.expanduser('~/ds_alt/performance_modelling/output/local_planning/*')
    parser.add_argument('-a', dest='alternative_base_run_folder',
                        help=f'Alternative folder in which the result of each run will be placed. Additionally to the base_run_folder. Defaults to {default_alternative_base_run_folder}.',
                        type=str,
                        default=default_alternative_base_run_folder,
                        required=False)

    parser.add_argument('-o', dest='output_dir_path',
                        help='Path of the output directory in which is written the dataframe (csv) with the results of all runs. Defaults to the directory of base_run_folder (where the run folders are)',
                        type=str,
                        default=None,
                        required=False)

    parser.add_argument('-j', dest='num_parallel_threads',
                        help=f'Number of parallel threads. Defaults to {default_num_parallel_threads}.',
                        type=int,
                        default=default_num_parallel_threads,
                        required=False)

    parser.add_argument('-s', dest='silent',
                        help='When set, reduce printed output.',
                        action='store_true',
                        required=False)

    parser.add_argument('-t', dest='print_timings',
                        help='When set, print timing of each metric (set num_parallel_threads to 1 t avoid parallel prints).',
                        action='store_true',
                        required=False)

    parser.add_argument('-c', dest='compress_benchmark_data',
                        help='When set, the directory benchmark_data in each run directory is compressed after computing metrics.'
                             'WARNING: If the data was previously compressed and this option is not set, the data will be left decompressed after computing the metrics. IT MAY TAKE A LOT OF SPACE ON DISK!',
                        action='store_true',
                        required=False)

    parser.add_argument('--compression_debug_info', dest='compression_debug_info',
                        help='When set, all information about the compression and deletion of files is printed.',
                        action='store_true',
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

    normal_run_folders = set(glob.glob(path.expanduser(args.base_run_folder)))
    alternative_run_folders = (glob.glob(path.expanduser(args.alternative_base_run_folder)))
    all_run_folders = list(normal_run_folders.union(alternative_run_folders))

    run_folders = sorted(list(filter(is_completed_run_folder, all_run_folders)))
    not_completed_run_folders = sorted(list(filter(is_not_completed_run_folder, glob.glob(path.expanduser(args.base_run_folder)))))

    if len(run_folders) == 0:
        if len(not_completed_run_folders):
            print_info("Runs not completed:")
            for not_completed_run_folder in not_completed_run_folders:
                print(not_completed_run_folder)

        print_info("Nothing to do.")
        sys.exit(0)

    global shared_progress
    shared_progress = multiprocessing.Value('i', 0)
    global shared_num_runs
    shared_num_runs = multiprocessing.Value('i', len(run_folders))
    global silent
    silent = args.silent
    global print_timings
    print_timings = args.print_timings
    global compress_benchmark_data
    compress_benchmark_data = args.compress_benchmark_data
    global compression_debug_info
    compression_debug_info = args.compression_debug_info
    global recompute_all_metrics
    recompute_all_metrics = args.recompute_all_metrics

    with multiprocessing.Pool(processes=args.num_parallel_threads) as pool:
        try:
            results_dfs_and_parameter_names = pool.starmap(parallel_compute_metrics, zip(run_folders))
            print_info("finished computing metrics")
        except KeyboardInterrupt:
            print_info("main: metrics computation interrupted")
            sys.exit()

        failed_metrics_computation_runs_ret = list(filter(lambda r_d: r_d[0] is None, results_dfs_and_parameter_names))

        results_dfs, run_parameter_names_lists, compute_directories = zip(*results_dfs_and_parameter_names)
        results_dfs, run_parameter_names_lists = filter(lambda d: d is not None, results_dfs), filter(lambda x: x is not None, run_parameter_names_lists)

        all_results_df = pd.concat(results_dfs, sort=False)
        all_results_df.to_csv(results_path, index=False)
        run_parameter_names = list(set(reduce(list.__add__, run_parameter_names_lists)))

        results_info = {'run_parameter_names': run_parameter_names, 'datetime': datetime.now().astimezone().isoformat()}
        with open(results_info_path, 'w') as results_info_file:
            yaml.dump(results_info, results_info_file)
        print_info("finished writing results")

        if len(failed_metrics_computation_runs_ret):
            _, _, failed_metrics_computation_runs = zip(*failed_metrics_computation_runs_ret)
            print_info(f"runs that failed metrics computation ({len(failed_metrics_computation_runs)}):")
            print("\t" + '\n\t'.join(failed_metrics_computation_runs))

    if len(not_completed_run_folders):
        print_info("runs not completed:")
        for not_completed_run_folder in not_completed_run_folders:
            print("\t" + not_completed_run_folder)


silent = False
print_timings = False
compress_benchmark_data = False
compression_debug_info = False
recompute_all_metrics = False
shared_progress = multiprocessing.Value('i', 0)
shared_num_runs = multiprocessing.Value('i', 0)

if __name__ == '__main__':
    main()
