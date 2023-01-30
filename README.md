# local_planning_performance_modelling

## Running the Experiments

To run the experiments using Docker, build the Docker image with the bash script [build_docker_image](https://github.com/AIRLab-POLIMI/local_planning_performance_modelling/blob/melodic-devel/docker/build_docker_image.sh).

Run the Docker image using the script [run_docker_image](https://github.com/AIRLab-POLIMI/local_planning_performance_modelling/blob/melodic-devel/docker/run_docker_image.sh) like so:
```shell
./run_docker_image.sh c0 0-3 # Arguments: Docker container name; CPU set available to the container (in this example, CPUs 0, 1, 2, 3 are available to the container)
```

In the container, use the following command to automatically run the experiments for all run parameter combinations (specified in [this configuration file](https://github.com/AIRLab-POLIMI/local_planning_performance_modelling/blob/melodic-devel/config/benchmark_configurations/grid_benchmark_all.yaml)):
```shell
rosrun local_planning_performance_modelling execute_grid_benchmark.py
```
To use arena-planner, run the following command:
```shell
rosrun local_planning_performance_modelling execute_grid_benchmark.py --rviz --ignore-previous-runs -d grid_benchmark_all_arena.yaml
```
To see additional options, run the following command in the container:
```shell
rosrun local_planning_performance_modelling execute_grid_benchmark.py -h
```

The output of the experiments will be placed in `~/ds/performance_modelling/output`.
Multiple containers can be run in parallel by setting different CPU sets to each container.

## Resources
- [Gazebo models for the robots and environments, and all the data necessary to execute experiments](https://github.com/AIRLab-POLIMI/performance_modelling_test_datasets/tree/local-planning-devel) (decompress files before use, using [this script](https://github.com/AIRLab-POLIMI/performance_modelling/blob/0.0.6/performance_modelling_py/environment/decompress_dataset_files.py))
- [Python package with automatic experiments execution and useful code](https://github.com/AIRLab-POLIMI/performance_modelling/tree/TODO) TODO set tag version
- [Gazebo plugins modified to add the odometric error model and ground truth topics](https://github.com/AIRLab-POLIMI/gazebo_ros_pkgs/tree/melodic-devel)
specifically, the [gazebo_ros_diff_drive](https://github.com/AIRLab-POLIMI/gazebo_ros_pkgs/blob/melodic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)
