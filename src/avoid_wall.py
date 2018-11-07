#!/usr/bin/env python

import sys
import rospy
import numpy as np
from math import cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

np.set_printoptions(precision=2)

orbit = 0


def calculate_lasers_range(data):
    '''Dynamic range intervals'''
    half_pi = np.pi/2
    initial_angle = 0
    if data.angle_min < -half_pi:
        default_min_angle = half_pi / data.angle_increment
        robot_initial_angle = -data.angle_min / data.angle_increment
        initial_angle = robot_initial_angle - default_min_angle
    if data.angle_max > np.pi/2:
        final_angle = data.angle_max / data.angle_increment

    laser_interval = (len(data.ranges) - initial_angle) / 5
    half_laser_interval = laser_interval / 2

    interval = [None] * 5
    interval[0] = np.mean(data.ranges[int(initial_angle):int(laser_interval)])
    for i in range(1,5):
        dirty_values = data.ranges[int(initial_angle + i * laser_interval - half_laser_interval):int(initial_angle + i * laser_interval + half_laser_interval) + 1]
        interval[i] = np.mean(np.nan_to_num(dirty_values))

    return interval


def laser_callback(data):
    global orbit

    # Configuration
    linear_vel = 0.1
    angular_vel = 0.4
    kd = 0.4  # KD = Keep Distance
    kdf = 0.30
    kds = 0.35

    inf = float('inf')

    left = -1
    going_left = -2
    right = 1
    going_right = 2

    # Lasers
    e, ne, n, nw, w = calculate_lasers_range(data)

    # Feedback
    rospy.loginfo("Orbit: %s, W : %s, NW: %s, N : %s, NE: %s, E : %s", orbit,
                  w, nw, n, ne, e)

    # Determine Action
    linear = 0
    angular = 0
    forward = False
    turn_left = False
    turn_right = False

    if (orbit == 0):  # Not orbiting yet
        if (w < kds):
            orbit = left
        elif (e < kds):
            orbit = right
        elif (nw < kd):
            orbit = going_left
            turn_right = True
        elif (ne < kd):
            orbit = going_right
            turn_left = True
        elif (n < kdf):
            # out of space but nothing on other beams, rotate somewhere
            orbit = going_left
            turn_right = True
        else:
            forward = True
    elif (orbit == going_left or orbit == going_right):
        if (w < kds):
            orbit = left
        elif (e < kds):
            orbit = right
        elif (orbit == going_left):
            turn_right = True
        elif (orbit == going_right):
            turn_left = True
    elif (orbit == left):
        if (n > kdf and (w > kds or e > kds)):
            forward = True
        if (w <= kds and e <= kds):
            turn_right = True
        elif (nw <= kd or ne <= kd):
            turn_right = True
        else:
            if (ne < kd or nw < kd or n < kdf):
                turn_right = True
            else:
                turn_left = True
    elif (orbit == right):
        if (n > kdf and (w > kds or e > kds)):
            forward = True
        if (w <= kds and e <= kds):
            turn_right = True
        elif (nw <= kd or ne <= kd):
            turn_right = True
        else:
            if (ne < kd or nw < kd or n < kdf):
                turn_right = True
            else:
                turn_left = True

    # right
    if (turn_left):
        angular += angular_vel
    if (turn_right):
        angular -= angular_vel
    if (forward):
        linear = linear_vel

    # Post
    vel_pub = rospy.Publisher(
        '/robot' + sys.argv[1] + '/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    vel_pub.publish(vel_msg)


def sonar_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.range)


def listeners():
    rospy.Subscriber('/robot' + sys.argv[1] + '/laser_0', LaserScan,
                     laser_callback)
    rospy.Subscriber('/robot' + sys.argv[1] + '/sonar_0', Range,
                     sonar_callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('avoid_wall_' + sys.argv[1], anonymous=True)
    listeners()

