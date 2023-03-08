#!/bin/bash

USER_NAME=$USER
USER_ID=$(id -u)
USER_GID=$(id -g)

if [ $# -eq 2 ]
  then
    echo "container id: $1"
    echo "cpu set: $2"
    mkdir -p ~/ds/performance_modelling
    mkdir -p ~/ds_alt/performance_modelling
    sudo docker run -ti --rm \
        --name "$USER_NAME"_"$1" \
        --user "$USER_ID":"$USER_GID" \
        --cpuset-cpus "$2" \
        -v ~/ds/performance_modelling/output:/home/"$USER_NAME"/ds/performance_modelling/output \
        -v ~/ds/performance_modelling/ros_logs:/home/"$USER_NAME"/.ros/log \
        -v ~/ds_alt/performance_modelling/output:/home/"$USER_NAME"/ds_alt/performance_modelling/output \
        -e DISPLAY=:1 \
        "$USER_NAME"/local_planning_performance_modelling_noetic_full:v3
  else
    echo "usage: $0 container_id cpu_set"
fi


# use -e DISPLAY=:1 to guarantee that pedsim starts by using a virtual screen