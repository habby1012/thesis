#!/bin/bash

bag_path=rosbag/carla_rosbag_one_camera

topics=$(ros2 bag info $bag_path | grep -E 'Topic:\s+/(vehicle|system)[^ ]*' | awk '{print $2}' | tr '\n' ' ')

sudo true

parallel --lb ::: \
  "sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER" bash -c 'ros2 run rmw_zenoh_cpp rmw_zenohd'" \
  "sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER" bash -c \"ros2 bag play $bag_path --topics $topics\"" \
  "sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER" bash -c 'ros2 run subscriber subscriber'"
