#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <bag_path>"
  exit 1
fi

bag_path=$1

topics=$(ros2 bag info $bag_path | grep -E 'Topic:\s+/(vehicle|system)[^ ]*' | awk '{print $2}' | tr '\n' ' ')

sudo true

parallel --lb ::: \
  "sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER" bash -c 'ros2 run rmw_zenoh_cpp rmw_zenohd'" \
  "sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER" bash -c \"ros2 bag play $bag_path --topics $topics -l\"" \
  "sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER" bash -c 'ros2 run subscriber subscriber'"
