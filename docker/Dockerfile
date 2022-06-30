FROM ros:melodic

ARG user_uid
ARG user_name
ARG user_gid
ARG user_group


# Install dependences
RUN \
apt install -y  \
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


# copy enrtypoint
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
  git clone --branch=master https://github.com/AIRLab-POLIMI/performance_modelling.git && \
  git clone --branch=noetic-devel-temp https://github.com/AIRLab-POLIMI/local_planning_performance_modelling.git && \
  git clone --branch=melodic-devel https://github.com/AIRLab-POLIMI/gazebo_ros_pkgs.git 
# gazebo_ros_pkgs needs to be cloned from the melodic-devel even if it is noetic
# TODO set appropriate performance_modelling tag version

# build workspace and install python user packages
RUN \
  . /opt/ros/noetic/setup.sh && \
  cd ~/w/catkin_ws && \
  catkin build -j 16

RUN \
  pip install pyquaternion && \
  pip3 install pyquaternion && \
  pip install networkx && \
  pip3 install networkx && \
  pip install scikit-image && \
  pip3 install scikit-image

RUN \
  cd ~/w/catkin_ws/src/performance_modelling/ && \
  ./install_dev.sh

RUN mkdir -p ~/.ros/log
RUN mkdir -p ~/ds/performance_modelling
RUN \
  cd ~/ds/performance_modelling && \
  git clone --branch=local-planning-devel https://github.com/AIRLab-POLIMI/performance_modelling_test_datasets.git test_datasets && \
  ~/w/catkin_ws/src/performance_modelling/performance_modelling_py/environment/decompress_dataset_files.py

# copy bashrc files
COPY .bashrc /home/$user_name/.bashrc
RUN sudo chown $user_name:$user_group /home/$user_name/.bashrc
COPY .bash_aliases /home/$user_name/.bash_aliases
RUN sudo chown $user_name:$user_group /home/$user_name/.bash_aliases
