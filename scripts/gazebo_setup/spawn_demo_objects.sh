#!/bin/bash

# Bash Script to spawn a table and AR marker in Gazebo

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/fetch_intro_ws/fetch_tasks/models



# Clear environment of objects
rosservice call /gazebo/delete_model "{model_name: 'table_1'}"
rosservice call /gazebo/delete_model "{model_name: 'table_2'}"
rosservice call /gazebo/delete_model "{model_name: 'object_1'}"

# Move Fetch to Start Position
rosrun fetch_tasks task2_transform_frame.py

# Spawn table
rosrun gazebo_ros spawn_model \
    -file ~/fetch_intro_ws/src/fetch_tasks/models/table_0/model1.sdf \
    -sdf \
    -model table_1 \
    -x 0.8 -y 0.3 -z 0.25

rosrun gazebo_ros spawn_model \
    -file ~/fetch_intro_ws/src/fetch_tasks/models/table_0/model1.sdf \
    -sdf \
    -model table_2 \
    -x 0.8 -y -0.3 -z 0.25

# Spawn object on table
rosrun gazebo_ros spawn_model \
    -file ~/fetch_intro_ws/src/fetch_tasks/models/object/model1.sdf \
    -sdf \
    -model object_1 \
    -x 0.65 -y -0.3 -z 0.56


echo "Table Spawned"