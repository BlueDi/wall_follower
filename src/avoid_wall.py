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

linear_vel = 0.1
angular_vel = 0.4
wall_distance = 0.4
wall_distance_forward = 0.30
wall_distance_side = 0.35

inf = float('inf')

left = -1
going_left = -2
right = 1
going_right = 2


def calculate_lasers_range(data):
    '''Dynamic range intervals'''
    half_pi = np.pi / 2
    initial_angle = 0
    if data.angle_min < -half_pi:
        default_min_angle = half_pi / data.angle_increment
        robot_initial_angle = -data.angle_min / data.angle_increment
        initial_angle = robot_initial_angle - default_min_angle
    if data.angle_max > np.pi / 2:
        final_angle = data.angle_max / data.angle_increment

    laser_interval = (len(data.ranges) - initial_angle) / 5
    half_laser_interval = laser_interval / 2

    interval = [None] * 5
    interval[0] = np.mean(data.ranges[int(initial_angle):int(laser_interval)])
    for i in range(1, 5):
        dirty_values = data.ranges[int(
            initial_angle + i * laser_interval - half_laser_interval
        ):int(initial_angle + i * laser_interval + half_laser_interval) + 1]
        interval[i] = np.mean(np.nan_to_num(dirty_values))

    return interval


def log_info(orbit, w, nw, n, ne, e):
    rospy.loginfo("Orbit: %s, W : %s, NW: %s, N : %s, NE: %s, E : %s", orbit,
                  w, nw, n, ne, e)


def laser_callback(data):
    global orbit

    e, ne, n, nw, w = calculate_lasers_range(data)

    log_info(orbit, w, nw, n, ne, e)

    linear = 0
    angular = 0
    forward = False
    turn_left = False
    turn_right = False

    if (orbit == 0):
        if (w < wall_distance_side):
            orbit = left
        elif (e < wall_distance_side):
            orbit = right
        elif (nw < wall_distance):
            orbit = going_left
            turn_right = True
        elif (ne < wall_distance):
            orbit = going_right
            turn_left = True
        elif (n < wall_distance_forward):
            orbit = going_left
            turn_right = True
        else:
            forward = True
    elif (orbit == going_left or orbit == going_right):
        if (w < wall_distance_side):
            orbit = left
        elif (e < wall_distance_side):
            orbit = right
        elif (orbit == going_left):
            turn_right = True
        elif (orbit == going_right):
            turn_left = True
    elif (orbit == left):
        if (n > wall_distance_forward
                and (w > wall_distance_side or e > wall_distance_side)):
            forward = True
        if (w <= wall_distance_side and e <= wall_distance_side):
            turn_right = True
        elif (nw <= wall_distance or ne <= wall_distance):
            turn_right = True
        else:
            if (ne < wall_distance or nw < wall_distance
                    or n < wall_distance_forward):
                turn_right = True
            else:
                turn_left = True                
    elif (orbit == right):
        if (n > wall_distance_forward
                and (w > wall_distance_side or e > wall_distance_side)):
            forward = True
        if (w <= wall_distance_side and e <= wall_distance_side):
            turn_left = True
        elif (nw <= wall_distance or ne <= wall_distance):
            turn_left = True
        else:
            if (ne < wall_distance or nw < wall_distance
                    or n < wall_distance_forward):
                turn_left = True
            else:
                turn_right = True


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
    pass


def listeners():
    rospy.Subscriber('/robot' + sys.argv[1] + '/laser_0', LaserScan,
                     laser_callback)
    rospy.Subscriber('/robot' + sys.argv[1] + '/sonar_0', Range,
                     sonar_callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('avoid_wall_' + sys.argv[1], anonymous=True)
    listeners()

