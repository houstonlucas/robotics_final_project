#!/usr/bin/python
import rospy
from geometry_msgs.msg  import Twist
import std_msgs.msg as stdmsgs
from turtlesim.msg import Pose


import argparse
import time
from math import asin, sin, cos, pi, sqrt, atan2


# Scaling factor of the attractive field
lambda_factor = 8.5

dt = 0.05

max_velocity = 2.0
simulate_turning_limit = False

max_turning_speed = pi / 0.5
max_turn_angle = max_turning_speed * dt


control = None
minimum_redirection_distance = 3.5
redirectScaler = 1
target_state = [9, 5, 0.0, 0.0]  # x y t v
obstacle_state = [target_state[0]/2, target_state[1], 0.0, 0.0]
robot_state = [0.0, 0.0, 0.0, 0.0]  # x y t v


def poseCallback(data):
    global robot_state
    x = data.x
    y = data.y
    t = data.theta
    v = data.linear_velocity
    robot_state = [x,y,t,v]

def otherTurtlePoseCallback(data):
    global obstacle_state
    x = data.x
    y = data.y
    t = data.theta
    v = data.linear_velocity
    obstacle_state = [x,y,t,v]


def controlEffortCallback(data):
    global control
    control = data.data

def normalize_angle(t):
    return ((t + pi) % (2 * pi)) - pi


def bound_dTheta(dTheta):
    dTheta = normalize_angle(dTheta)
    if -max_turn_angle > dTheta:
        return -max_turn_angle
    elif max_turn_angle < dTheta:
        return max_turn_angle
    else:
        return dTheta


def get_desired_cmd():
    theta_v = target_state[2]
    pv = target_state[3]
    pr = robot_state[2]
    qrv = get_distance(target_state, robot_state)
    phi = get_relative_heading(target_state, robot_state)

    if pr == 0.0:
        pr = 0.0001
    if pv == 0.0:
        if qrv < 0.1:
            return 0.0, theta_v
        pv = 0.0001

    u = lambda_factor * qrv

    vel = sqrt((pv ** 2) + 2 * u * pv * abs(cos(theta_v - phi)) + u ** 2)
    desired_vel = min(vel, max_velocity)
    desired_theta = phi + asin((pv * sin(theta_v - phi)) / desired_vel)

    if simulate_turning_limit:
        dTheta = normalize_angle(desired_theta - pr)
        if abs(dTheta) > pi / 2.0:
            desired_vel *= 0.5
        dTheta = bound_dTheta(dTheta)
        desired_theta = normalize_angle(pr + dTheta)

    return desired_vel, desired_theta


def get_distance(s1, s2):
    dx = s1[0] - s2[0]
    dy = s1[1] - s2[1]

    return sqrt(dy ** 2 + dx ** 2)

def get_nearest_equivalent(a1, a2):
    diff = a1-a2
    if diff > pi:
        a1 -= 2*pi
    elif diff < -pi:
        a1 += pi
    return a1


def get_relative_heading(s1, s2):
    x1, y1, _, _ = s1
    x2, y2, _, _ = s2
    dx = x1 - x2
    dy = y1 - y2
    return atan2(dy, dx)

def get_redirection_vector(s1, s2, k):
    theta = get_relative_heading(s1, s2)
    theta = theta - pi/2
    ds = get_distance(s1, s2)

    if ds > minimum_redirection_distance:
        return 0.0, 0.0

    V = k/ds
    # Vx = V*cos(theta)
    # Vy = V*sin(theta)
    return V, theta

def get_total_force():
    redVect, redTheta = get_redirection_vector(obstacle_state, robot_state, redirectScaler)
    targetVect, targetTheta = get_desired_cmd()

    redVectX = redVect*cos(redTheta)
    redVectY = redVect*sin(redTheta)
    targetVectX = targetVect*cos(targetTheta)
    targetVectY = targetVect*sin(targetTheta)

    vectX = redVectX + targetVectX
    vectY = redVectY + targetVectY
    theta = atan2(vectY, vectX)
    vect = sqrt(vectX*vectX + vectY*vectY)

    return vect, theta

def get_other_turtle_name(n):
    other_n = 2 if n == 1 else 1
    other_turtle = "/turtle{}".format(other_n)
    return other_turtle

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        '--robotNumber', '-n', required=False, type = int,
        default = 1, 
        help='Specifies which robot this is. (Which topics to listen to.)'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    turtle = "/turtle{}".format(args.robotNumber)
    other_turtle = get_other_turtle_name(args.robotNumber)

    rospy.init_node("pidHelper{}".format(args.robotNumber))
    rospy.Subscriber(turtle + "/pose", Pose, poseCallback)
    rospy.Subscriber(other_turtle + "/pose", Pose, otherTurtlePoseCallback)
    rospy.Subscriber(turtle + "/control_effort", stdmsgs.Float64, controlEffortCallback)
    statePub = rospy.Publisher(turtle + "/state", stdmsgs.Float64, queue_size = 10)
    setPointPub = rospy.Publisher(turtle + "/setpoint", stdmsgs.Float64, queue_size = 10)
    cmdVelPub = rospy.Publisher(turtle + "/cmd_vel", Twist, queue_size = 10)

    rate = rospy.Rate(60) #10hz
    
    while not rospy.is_shutdown():
        theta = robot_state[2]

        vel, desired_theta = get_total_force()
        if not theta is None:
            theta = get_nearest_equivalent(theta, desired_theta)
            statePub.publish(theta)
        if not control is None:
            myTwist = Twist()

            myTwist.angular.z = control
            myTwist.linear.x = vel
            cmdVelPub.publish(myTwist)

        setPointPub.publish(desired_theta)

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()