#!/bin/bash
# Bash script for deleting and adding models in Gazebo

echo "MODIFYING THE SIMULATION WORLD..."

if [ $# -eq 0 ]; then
    echo "Usage: $ ./switch_object.sh [MODEL TO DELETE] [MODEL TO ADD]"
    exit 1
fi

if [ "$1" != "NONE" ]
  then
  	# delete model
	rosservice call /gazebo/delete_model "{model_name: $1}"
fi

# add model
rosrun gazebo_ros spawn_model -file $(rospack find access_teleop)/models/$2/model.sdf -sdf -model $2 -x 3.8 -y 3 -z 0.83