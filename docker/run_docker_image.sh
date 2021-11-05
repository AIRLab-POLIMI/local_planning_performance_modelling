#!/bin/bash

# change USER in /home/${USER} if the host user name is not the same as the container user name

USER_NAME=$USER
USER_ID=$(id -u)
USER_GID=$(id -g)

if [ $# -eq 2 ]
  then
    echo "container id: $1"
    echo "cpu set: $2"
    sudo docker run -ti --rm \
        --name "$USER_NAME"_"$1" \
        --user "$USER_ID":"$USER_GID" \
        --cpuset-cpus "$2" \
        -v ~/datasets/private/performance_modelling:/home/"$USER_NAME"/ds/performance_modelling \
        -v ~/datasets/private/performance_modelling/ros_logs:/home/"$USER_NAME"/.ros/log \
        "$USER_NAME"/local_planning_performance_modelling:v1
  else
    echo "usage: $0 container_id cpu_set"
fi
