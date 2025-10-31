# Replace metadata table. This is to ensure all message flows go into the corresponding priority queue (zenoh priority 1 - 4)
cp /home/newslab/repos/thesis/src/subscriber/subscriber/config/backup/metadata_table.csv /home/newslab/repos/thesis/src/subscriber/subscriber/config/metadata_table.csv

# Config tc qdisc. This is to ensure all message flows go into the corresponding priority queue (zenoh priority -> socket priority -> hardware queue)
sudo tc qdisc del dev enp4s0 root

sudo ip link set dev enp4s0 down
sudo ip link set dev enp4s0 up

sleep 5

sudo tc qdisc replace dev enp4s0 root handle 100: taprio \
  num_tc 4 \
  map 3 3 3 3 3 0 2 1 3 3 3 3 3 3 3 3 \
  queues 1@0 1@1 1@2 1@3 \
  base-time 0 cycle-time 200000 \
  sched-entry S 0x2 10000  \
  sched-entry S 0x4 20000  \
  sched-entry S 0x9 170000 \
  flags 0x2

tc qdisc

# Set Vlan. This is to ensure host's hardware queues map to corresponding switch's queues.
sudo ip link set dev vlan2 type vlan egress 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7

# Set link speed = 100Mb/s
sudo ethtool -s enp4s0 speed 100

# Sleep some time, since change qdisc may affect ptp synchronization
sleep 30

# test ptp
sudo systemctl status ptp4l
sudo systemctl status phc2sys
