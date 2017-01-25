#!/bin/bash
masterip=192.168.0.x
slaveip=$(hostname -I)
export ROS_IP=$slaveip
export ROS_MASTER_URI=http://$masterip:11311/
sudo date --set="$(ssh master@$masterip date)"
