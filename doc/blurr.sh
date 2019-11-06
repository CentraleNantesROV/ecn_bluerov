#!/bin/bash
unset ROS_MASTER_URI
unset ROS_IP

export ROS_MASTER_URI=http://blurr.local:11311
export ROS_IP=$(ip -4 addr show wlo1 | grep -oP "(?<=inet ).*(?=/)")

#enp0s25
