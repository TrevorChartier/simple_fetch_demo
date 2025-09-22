#!/bin/bash

# Bash Script to spawn AR Marker in Gazebo for Task3

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/fetch_intro_ws/fetch_tasks/models



# Clear environment of objects
rosservice call /gazebo/delete_model "{model_name: 'ar_marker_0'}"

# Move Fetch to Start Position
rosrun fetch_tasks task1_base_pose.py

# Spawn Marker 
rosrun gazebo_ros spawn_model -file  ~/fetch_intro_ws/src/fetch_tasks/models/ar_marker_0/model.sdf -sdf -model ar_marker_0 -x 0.7 -y 0.0 -z 1.2

echo "Marker Spawned"