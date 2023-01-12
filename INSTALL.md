## Installation steps
0) Create a workspace
```shell
mkdir -p ~/w/catkin_ws/src
```

1) Clone the following repos (gazebo_ros_pkgs needs to be cloned from the melodic-devel branch even if the ROS distro is noetic)
```shell
cd ~/w/catkin_ws/src && \
git clone --branch=master https://github.com/AIRLab-POLIMI/performance_modelling.git && \
git clone --branch=noetic-devel-temp https://github.com/AIRLab-POLIMI/local_planning_performance_modelling.git && \
git clone --branch=melodic-devel https://github.com/AIRLab-POLIMI/gazebo_ros_pkgs.git 
```
2) Build workspace and install python user packages

```shell
. /opt/ros/noetic/setup.sh && \
cd ~/w/catkin_ws && \
catkin build -j 16

pip install pyquaternion && \
pip3 install pyquaternion && \
pip install networkx && \
pip3 install networkx && \
pip install scikit-image && \
pip3 install scikit-image
```
3)
```shell
cd ~/w/catkin_ws/src/performance_modelling/ && \
./install_dev.sh
```
4) 
```shell
mkdir -p ~/.ros/log
mkdir -p ~/ds/performance_modelling
cd ~/ds/performance_modelling && \
  git clone --branch=local-planning-ROS1-new https://github.com/AIRLab-POLIMI/performance_modelling_test_datasets.git test_datasets && \
  ~/w/catkin_ws/src/performance_modelling/performance_modelling_py/environment/decompress_dataset_files.py
```  
5)  
```shell
pip install pillow==8.3.2
```
6) Install this numpy version to avoid compatibility conflicts with panda
```shell
pip install numpy==1.19.2 
```
7) Install this package so that Python 3.8 is set to default instead of 2.7
```shell
sudo apt install python-is-python3
```
8) Install packages
```shell
sudo apt install -y  \
	bash-completion \
	htop \
	git-extras \
	nano \
	python3-pandas \
	python3-termcolor \
	python3-pip \
	python3-shapely \
	python3-yaml \
	python3-skimage \
	python3-sklearn \
	python-psutil \
	python3-psutil \
	ros-noetic-catkin \
	ros-noetic-gazebo-ros \
	ros-noetic-gazebo-plugins \
	ros-noetic-control-toolbox \
	ros-noetic-controller-manager \
	ros-noetic-transmission-interface \
	ros-noetic-joint-limits-interface \
	ros-noetic-robot-state-publisher \
	ros-noetic-map-server \
	ros-noetic-slam-toolbox \
	ros-noetic-gmapping \
	ros-noetic-hector-slam \
	ros-noetic-amcl \
	ros-noetic-move-base \
	ros-noetic-global-planner \
	ros-noetic-dwa-local-planner \
	ros-noetic-tf-conversions \
	ros-noetic-tf2-sensor-msgs 
```
9) Clone arena-rosnav-3D repo (TODO change to that it should be cloning my repo instead of the official)
```shell
cd ~/w/catkin_ws/src && \
git clone https://github.com/ignc-research/arena-rosnav-3D.git
```
10) Install packages for arena
```shell
sudo apt-get update && sudo apt-get install -y \
	libopencv-dev \
	liblua5.2-dev \
	screen \
	python3-rosdep \
	python3-rosinstall \
	python3-rosinstall-generator \
	build-essential \
	python3-rospkg-modules \
	ros-noetic-navigation \
	ros-noetic-teb-local-planner \
	ros-noetic-mpc-local-planner \
	libarmadillo-dev \
	ros-noetic-nlopt \
	ros-noetic-turtlebot3-description \
	ros-noetic-turtlebot3-navigation \
	ros-noetic-lms1xx \
	ros-noetic-velodyne-description 
```
11) Additional packages for arena
```shell		
pip3 install pyyaml catkin_pkg gym netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml
```
