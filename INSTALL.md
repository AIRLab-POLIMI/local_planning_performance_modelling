## Installation steps
1)
```shell
pip install pillow==8.3.2
```
2) Install this numpy version to avoid compatibility conflicts with panda
```shell
pip install numpy==1.19.2 
```
3) Install this package so that Python 3.8 is set to default instead of 2.7
```shell
sudo apt install python-is-python3
```
4) Install packages
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
5) Clone arena-rosnav-3D repo
```shell
cd ~/w/catkin_ws/src && \
  git clone https://github.com/ignc-research/arena-rosnav-3D.git
```
6) Install packages for arena
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
7) Additional packages from arena
```shell		
pip3 install pyyaml catkin_pkg gym netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml
```
