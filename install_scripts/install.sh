#!/bin/bash

mkdir -p ~/w/ros2_ws/src
cd ~/w/ros2_ws/src

git clone branch=eloquent-plugins-only https://github.com/Enri2077/gazebo_ros_pkgs.git
git clone https://github.com/Enri2077/localization_performance_modelling.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/Enri2077/performance_modelling.git

cd ~/w/ros2_ws/src/localization_performance_modelling
./install_dependences.sh  # Installs dependencies with apt and pip

cd ~/w/ros2_ws/src/performance_modelling
./install_dependencies.sh  # Installs dependencies with apt and pip
./install_dev.sh  # Installs the performance_modelling_py python package by linking to the source

cd ~/w/ros2_ws
colcon build --symlink-install --continue-on-error

