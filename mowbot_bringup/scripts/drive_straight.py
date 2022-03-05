#!/usr/bin/env python

import sys
from tkinter import W
from turtle import backward
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2

class DriveStraight():
    def __init__(self, distance, cmd_vel:W):
        self.distance = distance
        self.rate = 10              # loop rate
        self.linear_speed = 0.25    # m/s
        self.slew_rate = 0.5 / self.rate    # m/s^2 per loop
    def print(self):
        print('driving straight for {} m'.format(self.distance))

    def run(self):
        pass

def shutdown(self):
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    self.cmd_vel.publish(Twist())
    rospy.sleep(1)

def usage():
    Usage: drive_straight.py <distance> - drive the specified distance forward or backward
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        usage()

    distance = float(sys.argv[1])
    rospy.init_node('move', anonymous=False)

    # Set rospy to execute a shutdown function when exiting       
    rospy.on_shutdown(shutdown)

    # Publisher to control the robot's speed
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        

    try:
        print('driving {} m'.format(distance))
        m = DriveStraight(distance, cmd_vel)
        m.print()
        # m.run()

    except Exception as e:
        print(e)
        rospy.loginfo("Out-and-Back node terminated.")
