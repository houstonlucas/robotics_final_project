#!/bin/bash
roslaunch multirobotPID.launch turtle2x:=$6 turtle2y:=$7 turtle2theta:=$8 &
sleep 1s
roslaunch teleportpenset.launch turtle1x:=$1 turtle1y:=$2 turtle1theta:=$3 &
sleep 1s
roslaunch penset.launch target1x:=$4 target1y:=$5 target2x:=$9 target2y:=$10