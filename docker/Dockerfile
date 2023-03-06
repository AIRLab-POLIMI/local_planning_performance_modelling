FROM osrf/ros:noetic-desktop-full

ARG user_uid
ARG user_name
ARG user_gid
ARG user_group


# Install dependences
RUN apt update && \
apt install -y  \
	htop \
	git-extras \
	nano \
	python3-pandas \
	python3-termcolor \
	python3-pip \
	python3-shapely \
	python3-yaml \
	python3-catkin-tools \
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

# Install dependences (arena)
RUN apt update && \
apt install -y  \
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
	ros-noetic-velodyne-description \
	python-is-python3 \
  	ros-noetic-pcl-conversions

# copy entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh


# switch to user
RUN \
  groupadd --gid $user_gid $user_group && \
  useradd --uid $user_uid --gid $user_group --shell /bin/bash --create-home $user_name
RUN adduser $user_name sudo
RUN echo "$user_name ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER $user_name:$user_group


# clone repos
RUN mkdir -p ~/w/catkin_ws/src
RUN \
  cd ~/w/catkin_ws/src && \
  git clone --branch=1.0.2  https://github.com/AIRLab-POLIMI/performance_modelling.git && \
  git clone --branch=noetic-devel https://github.com/AIRLab-POLIMI/local_planning_performance_modelling.git && \
  git clone --branch=local-planning-performance-modelling-noetic https://github.com/AIRLab-POLIMI/gazebo_ros_pkgs.git && \
  git clone --depth 1 --branch=local-planning-performance-modelling-noetic https://github.com/AIRLab-POLIMI/arena-rosnav-3D.git && \
  git clone --branch=local-planning-performance-modelling-noetic https://github.com/AIRLab-POLIMI/pedsim_ros.git

# build workspace and install python user packages
RUN \
  . /opt/ros/noetic/setup.sh && \
  cd ~/w/catkin_ws && \
  catkin_make -j 2

RUN \
  pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed \
  && pip3 install pyquaternion==0.9.9 networkx==3.0 scikit-learn==1.2.1 scikit-image==0.19.3 pandas==0.25.3 numpy==1.19.2 scipy==1.3.3 pillow==8.3.2 psutil==5.9.4

# install python packages for arena
RUN pip3 install pyyaml catkin_pkg gym netifaces pathlib filelock pyqt5 mpi4py torch lxml defusedxml 

# install virtual screen to support pedsim which uses qt
RUN sudo apt update && sudo apt install -y xvfb

RUN \
  cd ~/w/catkin_ws/src/performance_modelling/ && \
  ./install_dev.sh

RUN mkdir -p ~/.ros/log
RUN mkdir -p ~/ds/performance_modelling
RUN mkdir -p ~/ds_alt/performance_modelling
RUN \
  cd ~/ds/performance_modelling && \
  git clone --depth 1 --branch=local-planning-ROS1-new https://github.com/AIRLab-POLIMI/performance_modelling_test_datasets.git test_datasets && \
  ~/w/catkin_ws/src/performance_modelling/performance_modelling_py/environment/decompress_dataset_files.py

# copy bashrc files
COPY .bashrc /home/$user_name/.bashrc
RUN sudo chown $user_name:$user_group /home/$user_name/.bashrc
COPY .bash_aliases /home/$user_name/.bash_aliases
RUN sudo chown $user_name:$user_group /home/$user_name/.bash_aliases
