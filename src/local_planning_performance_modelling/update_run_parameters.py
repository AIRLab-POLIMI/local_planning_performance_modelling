#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import glob
import argparse
import shutil

import yaml
from os import path

from performance_modelling_py.utils import print_info, print_error


def update_run_parameters(base_run_folder_path):

    base_run_folder = path.expanduser(base_run_folder_path)

    if not path.isdir(base_run_folder):
        print_error("update_run_parameters: base_run_folder does not exists or is not a directory".format(base_run_folder))
        return None, None

    run_folders = sorted(list(filter(path.isdir, glob.glob(path.abspath(base_run_folder) + '/*'))))

    print_info("update_run_parameters: reading run data")
    no_output = True
    for i, run_folder in enumerate(run_folders):
        updated_run_info_file_path = path.join(run_folder, "run_info.yaml")
        original_run_info_file_path = path.join(run_folder, "run_info_original.yaml")

        if not path.exists(updated_run_info_file_path) and not path.exists(original_run_info_file_path):
            print_error("update_run_parameters: run_info file does not exists [{}]".format(run_folder))
            no_output = False
            continue

        # backup run_info if there is not already a backup of the original
        # do not overwrite the original, since it will always be used to create the updated run_info
        if not path.exists(original_run_info_file_path):
            shutil.copyfile(updated_run_info_file_path, original_run_info_file_path)

        with open(original_run_info_file_path) as original_run_info_file:
            run_info = yaml.safe_load(original_run_info_file)

        if 'run_parameters' not in run_info:
            print_error("update_run_parameters: run_parameters not in run_info [{}]".format(run_folder))
            no_output = False
            continue

        # remove alpha_n parameters
        if 'alpha_1' in run_info['run_parameters']:
            del run_info['run_parameters']['alpha_1']
        if 'alpha_2' in run_info['run_parameters']:
            del run_info['run_parameters']['alpha_2']
        if 'alpha_3' in run_info['run_parameters']:
            del run_info['run_parameters']['alpha_3']
        if 'alpha_4' in run_info['run_parameters']:
            del run_info['run_parameters']['alpha_4']

        with open(updated_run_info_file_path, 'w') as updated_run_info_file:
            yaml.dump(run_info, updated_run_info_file, default_flow_style=False)

        print_info("update_run_parameters: {}%".format(int((i + 1)*100/len(run_folders))), replace_previous_line=no_output)
        no_output = True


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description='Updates run parameters.')

    default_base_run_folder = "~/ds/performance_modelling/output/test_local_planning/"
    parser.add_argument('-r', dest='base_run_folder',
                        help='Folder containing the run folders. Defaults to {}'.format(default_base_run_folder),
                        type=str,
                        default=default_base_run_folder,
                        required=False)

    args = parser.parse_args()
    update_run_parameters(args.base_run_folder)


if __name__ == '__main__':
    main()
