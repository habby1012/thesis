# Replace metadata table. This is to ensure all message flows go into the corresponding priority queue (zenoh priority 1 - 4)
cp /home/newslab/repos/thesis/src/subscriber/subscriber/config/backup/metadata_table.csv /home/newslab/repos/thesis/src/subscriber/subscriber/config/metadata_table.csv

# Config tc qdisc. This is to ensure all message flows go into the same hardware queue (queue 0) (zenoh priority -> socket priority -> hardware queue)
sudo tc qdisc del dev enp4s0 root
sudo tc qdisc replace dev enp4s0 root mqprio   num_tc 1 map 0 0 0 0 0 0 0 0   queues 1@0 hw 0
tc qdisc

# Set Vlan. This is to ensure host's hardware queues map to one switch's queue.
sudo ip link set dev vlan2 type vlan egress 0:2 1:2 2:2 3:2 4:2 5:2 6:2 7:2

# test ptp
sudo systemctl status ptp4l
sudo systemctl status phc2sys
