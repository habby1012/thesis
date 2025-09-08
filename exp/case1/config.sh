# set vlan
sudo ip link set dev vlan2 type vlan egress 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7
sudo tc qdisc del dev enp4s0 root

# replace metadata table
cp ../../src/subscriber/subscriber/config/backup/INTER_carla_one_camera_DEFAULT.csv ../../src/subscriber/subscriber/config/INTER_carla_one_camera.csv

# config tc qdisc
sudo tc qdisc del dev enp4s0 root
sudo tc qdisc replace dev enp4s0 root mqprio   num_tc 1 map 0 0 0 0 0 0 0 0   queues 1@0 hw 0
tc qdisc

# test ptp
sudo systemctl status ptp4l
sudo systemctl status phc2sys
