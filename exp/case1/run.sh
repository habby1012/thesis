#!/bin/bash
set -e

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
  echo "Usage: $0 <bag_path> <output_dir> <timeout>"
  exit 1
fi

BAG_PATH=$1
OUT_DIR=$2
TIMEOUT=$3

mkdir -p "$OUT_DIR"

timeout "$TIMEOUT" ../../play_rosbag_with_rmw_zenoh.sh "$BAG_PATH" || true

sleep 10

mv ../../result/zenoh_latency.csv "$OUT_DIR/" 
