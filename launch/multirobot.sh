#!/bin/bash
roslaunch multirobotPID.launch turtle2x:=$1 turtle2y:=$2 turtle2theta:=$3 &
sleep 1s
roslaunch teleportpenset.launch turtle1x:=$4 turtle1y:=$5 turtle1theta:=$6 &
sleep 1s
roslaunch penset.launch