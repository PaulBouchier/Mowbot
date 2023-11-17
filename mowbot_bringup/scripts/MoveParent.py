#!/usr/bin/env python

'''
This is the parent class of several move child classes. It provides support commont to all
'''
import sys
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2

loop_rate = 10       # loop rate
speed_default = 0.35    # driving speed, fwd or back
low_speed_default = 0.15
vel_slew_rate = 0.5 / loop_rate  # m/s^2 per loop
rot_speed_default = 0.5    # rotating speed, rad/s
low_rot_speed_default = 0.25
rot_slew_rate = 0.5 / loop_rate  # rad/s^2 per loop

class MoveParent():
    def __init__(self, cmd_vel):
        self.once = True
        self.speed = speed_default
        self.full_speed = speed_default
        self.low_speed = low_speed_default
        self.rot_speed = rot_speed_default
        self.full_rot_speed = rot_speed_default
        self.low_rot_speed = low_rot_speed_default
        self.odom = Odometry()

        # Publisher to control the robot's speed
        self.cmd_vel = cmd_vel

        # subscribers to robot data
        self.odom_extra = OdomExtra()
        self.platform_data = PlatformData()
        self.odometry = Odometry()
        rospy.Subscriber("/odom_extra", OdomExtra, self.odom_extra_callback, queue_size=1)
        rospy.Subscriber("/platform_data", PlatformData, self.platform_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        
        self.move_cmd = Twist()
        time.sleep(0.1)      #  wait for /odom_extra to populate odometer

    def slew_vel(self, to):
        return self.slew(self.platform_data.commandedLinear, to, vel_slew_rate)

    def slew_rot(self, to):
        return self.slew(self.platform_data.commandedAngular, to, rot_slew_rate)

    def slew(self, current, to, slew_rate):
        diff = to - current
        if diff > slew_rate:
            return current + slew_rate
        if diff < -slew_rate:
            return current - slew_rate
        return to

    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def odom_extra_callback(self, odom_extra_msg):
        self.odom_extra = odom_extra_msg

    def platform_callback(self, platform_data_msg):
        self.platform_data = platform_data_msg
