#!/usr/bin/python
import rospy
from geometry_msgs.msg  import Twist
import std_msgs.msg as stdmsgs
from turtlesim.msg import Pose


import time
from math import sin, pi


theta = None
control = None

def poseCallback(data):
    global theta
    theta = data.theta

def controlEffortCallback(data):
    global control
    control = data.data


def main():
    rospy.init_node("pidHelper")
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)
    rospy.Subscriber("/robot1/control_effort", stdmsgs.Float64, controlEffortCallback)
    statePub = rospy.Publisher("/robot1/state", stdmsgs.Float64, queue_size = 10)
    setPointPub = rospy.Publisher("/robot1/setpoint", stdmsgs.Float64, queue_size = 10)
    cmdVelPub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)

    rate = rospy.Rate(60) #10hz
    
    while not rospy.is_shutdown():
        if not theta is None:
            statePub.publish(theta)
        if not control is None:
            myTwist = Twist()
            myTwist.angular.z = control
            myTwist.linear.x = 0.5
            cmdVelPub.publish(myTwist)

        t = time.time()
        target_heading = pi*1.5*sin(t)
        setPointPub.publish(target_heading)

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()