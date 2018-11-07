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
laser_sensors = {'w': 0, 'nw': 0, 'n': 0, 'ne': 0, 'e': 0}

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
    global laser_sensors
    half_pi = np.pi / 2
    initial_angle = 0
    final_angle = 0
    if data.angle_min < -half_pi:
        default_min_angle = half_pi / data.angle_increment
        robot_initial_angle = -data.angle_min / data.angle_increment
        initial_angle = robot_initial_angle - default_min_angle
    if data.angle_max > np.pi / 2:
        default_max_angle = half_pi / data.angle_increment
        robot_final_angle = data.angle_max / data.angle_increment
        final_angle = robot_final_angle - default_max_angle

    laser_interval = (len(data.ranges) - initial_angle - final_angle) / 5
    half_laser_interval = laser_interval / 2

    interval = [None] * 5
    interval[0] = np.mean(data.ranges[int(initial_angle):int(laser_interval)])
    for i in range(1, 5):
        dirty_values = data.ranges[int(
            initial_angle + i * laser_interval - half_laser_interval
        ):int(initial_angle + i * laser_interval + half_laser_interval) + 1]
        interval[i] = np.mean(np.nan_to_num(dirty_values))

    laser_sensors['e'] = interval[0]
    laser_sensors['ne'] = interval[1]
    laser_sensors['n'] = interval[2]
    laser_sensors['nw'] = interval[3]
    laser_sensors['w'] = interval[4]


def log_info():
    '''Initial orbit state'''
    global orbit, laser_sensors
    orbit_values = {-2: 'Going Left', -1: 'Left', 0: 'Undefined', 1: 'Right', 2: 'Going Right'}
    rospy.loginfo("Orbit: %s, W : %s, NW: %s, N : %s, NE: %s, E : %s", orbit_values[orbit],
                  laser_sensors['w'], laser_sensors['nw'], laser_sensors['n'],
                  laser_sensors['ne'], laser_sensors['e'])


def create_velocity_message(turn_left, turn_right, forward):
    angular = 0
    linear = 0
    if (turn_left):
        angular += angular_vel
    if (turn_right):
        angular -= angular_vel
    if (forward):
        linear = linear_vel
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    return vel_msg


def publish_velocity_message(vel_msg):
    vel_pub = rospy.Publisher(
        '/robot' + sys.argv[1] + '/cmd_vel', Twist, queue_size=10)
    vel_pub.publish(vel_msg)


def laser_callback(data):
    global orbit, laser_sensors

    calculate_lasers_range(data)

    log_info()

    linear = 0
    angular = 0
    forward = False
    turn_left = False
    turn_right = False

    if (orbit == 0):
        if (laser_sensors['w'] < wall_distance_side):
            orbit = left
        elif (laser_sensors['e'] < wall_distance_side):
            orbit = right
        elif (laser_sensors['nw'] < wall_distance):
            orbit = going_left
            turn_right = True
        elif (laser_sensors['ne'] < wall_distance):
            orbit = going_right
            turn_left = True
        elif (laser_sensors['n'] < wall_distance_forward):
            orbit = going_left
            turn_right = True
        else:
            forward = True
    elif (orbit == going_left or orbit == going_right):
        if (laser_sensors['w'] < wall_distance_side):
            orbit = left
        elif (laser_sensors['e'] < wall_distance_side):
            orbit = right
        elif (orbit == going_left):
            turn_right = True
        elif (orbit == going_right):
            turn_left = True
    elif (orbit == left):
        if (laser_sensors['n'] > wall_distance_forward
                and (laser_sensors['w'] > wall_distance_side
                     or laser_sensors['e'] > wall_distance_side)):
            forward = True
        if (laser_sensors['w'] <= wall_distance_side
                and laser_sensors['e'] <= wall_distance_side):
            turn_right = True
        elif (laser_sensors['nw'] <= wall_distance
              or laser_sensors['ne'] <= wall_distance):
            turn_right = True
        else:
            if (laser_sensors['ne'] < wall_distance
                    or laser_sensors['nw'] < wall_distance
                    or laser_sensors['n'] < wall_distance_forward):
                turn_right = True
            else:
                turn_left = True               
    elif (orbit == right):
        if (laser_sensors['n'] > wall_distance_forward
                and (laser_sensors['w'] > wall_distance_side
                     or laser_sensors['e'] > wall_distance_side)):
            forward = True
        if (laser_sensors['w'] <= wall_distance_side
                and laser_sensors['e'] <= wall_distance_side):
            turn_left = True
        elif (laser_sensors['nw'] <= wall_distance
              or laser_sensors['ne'] <= wall_distance):
            turn_left = True
        else:
            if (laser_sensors['ne'] < wall_distance
                    or laser_sensors['nw'] < wall_distance
                    or laser_sensors['n'] < wall_distance_forward):
                turn_left = True
            else:
                turn_right = True

    vel_msg = create_velocity_message(turn_left, turn_right, forward)
    
    publish_velocity_message(vel_msg)


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

