#!/bin/bash
roslaunch multirobotPID.launch turtle2x:=$4 turtle2y:=$5 turtle2theta:=$6 &
sleep 1s
roslaunch teleportpenset.launch turtle1x:=$1 turtle1y:=$2 turtle1theta:=$3 &
sleep 1s
roslaunch penset.launch