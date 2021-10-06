FROM ros:foxy

ARG user_uid
ARG user_name
ARG user_gid
ARG user_group

# switch to user
RUN \
  groupadd --gid $user_gid $user_group && \
  useradd --uid $user_uid --gid $user_group --shell /bin/bash --create-home $user_name
RUN adduser $user_name sudo
RUN echo "$user_name ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER $user_name:$user_group

# Install dependences
RUN \
  sudo apt update && \
  sudo apt install -y \
    bash-completion \
    htop \
    nano \
    python3-pandas \
    python3-yaml \
    python3-termcolor \
    python3-psutil \
    python3-pip \
    python3-skimage \
    python3-sklearn \
    python3-colcon-common-extensions \
    ros-foxy-rviz2 \
    ros-foxy-gazebo-*

# clone repos
RUN mkdir -p ~/w/ros2_ws/src
RUN \
  cd ~/w/ros2_ws/src && \
  git clone --branch=master https://github.com/AIRLab-POLIMI/performance_modelling.git && \
  git clone --branch=foxy-devel https://github.com/AIRLab-POLIMI/local_planning_performance_modelling.git && \
  git clone --branch=ros2 https://github.com/rst-tu-dortmund/costmap_converter.git && \
  git clone --branch=foxy-devel https://github.com/rst-tu-dortmund/teb_local_planner.git

RUN \
  rosdep update && \
  cd ~/w/ros2_ws && \
  rosdep install --from-paths src --ignore-src -r -y

# build workspace and install python user packages
RUN \
  . /opt/ros/foxy/setup.sh && \
  cd ~/w/ros2_ws && \
  MAKEFLAGS="-j 4" colcon build --parallel-workers 4

RUN \
  pip3 install pyquaternion && \
  pip3 install networkx && \
  pip3 install scikit-image

RUN \
  cd ~/w/ros2_ws/src/performance_modelling/ && \
  ./install_dev.sh

RUN mkdir -p ~/ds/performance_modelling
RUN mkdir -p ~/.ros/log

# copy bashrc files
COPY .bashrc /home/$user_name/.bashrc
RUN sudo chown $user_name:$user_name /home/$user_name/.bashrc
COPY .bash_aliases /home/$user_name/.bash_aliases
RUN sudo chown $user_name:$user_name /home/$user_name/.bash_aliases
