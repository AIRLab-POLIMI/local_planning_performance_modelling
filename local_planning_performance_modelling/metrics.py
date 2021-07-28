#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob
import os
from os import path
import yaml

from performance_modelling_py.utils import print_info
from performance_modelling_py.metrics.localization_metrics import trajectory_length_metric, relative_localization_error_metrics, absolute_localization_error_metrics
from performance_modelling_py.metrics.computation_metrics import cpu_and_memory_usage_metrics
from performance_modelling_py.visualisation.trajectory_visualisation import save_trajectories_plot


def compute_metrics(run_output_folder):
    metrics_result_dict = dict()

    # localization metrics
    estimated_correction_poses_path = path.join(run_output_folder, "benchmark_data", "estimated_correction_poses.csv")
    estimated_poses_path = path.join(run_output_folder, "benchmark_data", "estimated_poses.csv")
    ground_truth_poses_path = path.join(run_output_folder, "benchmark_data", "ground_truth_poses.csv")

    logs_folder_path = path.join(run_output_folder, "logs")

    print_info("trajectory_length")
    metrics_result_dict['trajectory_length'] = trajectory_length_metric(ground_truth_poses_path)

    print_info("relative_localization_correction_error")
    metrics_result_dict['relative_localization_correction_error'] = relative_localization_error_metrics(path.join(logs_folder_path, "relative_localisation_correction_error"), estimated_correction_poses_path, ground_truth_poses_path)
    print_info("relative_localization_error")
    metrics_result_dict['relative_localization_error'] = relative_localization_error_metrics(path.join(logs_folder_path, "relative_localisation_error"), estimated_poses_path, ground_truth_poses_path)

    print_info("absolute_localization_correction_error")
    metrics_result_dict['absolute_localization_correction_error'] = absolute_localization_error_metrics(estimated_correction_poses_path, ground_truth_poses_path)
    print_info("absolute_localization_error")
    metrics_result_dict['absolute_localization_error'] = absolute_localization_error_metrics(estimated_poses_path, ground_truth_poses_path)

    # computation metrics
    print_info("cpu_and_memory_usage")
    ps_snapshots_folder_path = path.join(run_output_folder, "benchmark_data", "ps_snapshots")
    metrics_result_dict['cpu_and_memory_usage'] = cpu_and_memory_usage_metrics(ps_snapshots_folder_path)

    # write metrics
    metrics_result_folder_path = path.join(run_output_folder, "metric_results")
    metrics_result_file_path = path.join(metrics_result_folder_path, "metrics.yaml")
    if not path.exists(metrics_result_folder_path):
        os.makedirs(metrics_result_folder_path)
    with open(metrics_result_file_path, 'w') as metrics_result_file:
        yaml.dump(metrics_result_dict, metrics_result_file, default_flow_style=False)

    # visualisation
    print_info("visualisation")
    visualisation_output_folder = path.join(run_output_folder, "visualisation")
    save_trajectories_plot(visualisation_output_folder, estimated_poses_path, estimated_correction_poses_path, ground_truth_poses_path)


if __name__ == '__main__':
    run_folders = list(filter(path.isdir, glob.glob(path.expanduser("~/ds/performance_modelling_output/test_localization/*"))))
    # last_run_folder = sorted(run_folders, key=lambda x: path.getmtime(x))[-1]
    # print("last run folder:", last_run_folder)
    for progress, run_folder in enumerate(run_folders):
        print_info("main: compute_metrics {}% {}".format((progress + 1)*100/len(run_folders), run_folder))
        compute_metrics(path.expanduser(run_folder))
