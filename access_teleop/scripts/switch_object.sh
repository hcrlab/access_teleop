#!/bin/bash
# Bash script for deleting and adding models in Gazebo

echo "MODIFYING THE SIMULATION WORLD..."
# delete model
rosservice call /gazebo/delete_model "{model_name: $1}"
# add model
rosrun gazebo_ros spawn_model -file $(rospack find access_teleop)/models/$2/model.sdf -sdf -model $2 -x 3.8 -y 3 -z 0.83

