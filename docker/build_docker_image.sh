#!/bin/bash

USER_NAME=$USER
USER_ID=$(id -u)
USER_GID=$(id -g)
USER_GROUP=$(id -g -n "$USER")

sudo docker build \
  --build-arg user_uid="$USER_ID" \
  --build-arg user_name="$USER_NAME" \
  --build-arg user_gid="$USER_GID" \
  --build-arg user_group="$USER_GROUP" \
  -t "$USER_NAME"/local_planning_performance_modelling_noetic_full:v3 .

#--no-cache \