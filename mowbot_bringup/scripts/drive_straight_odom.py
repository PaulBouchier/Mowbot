#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2

loop_rate = 10       # loop rate
speed = 0.25    # driving speed, fwd or back
vel_slew_rate = 0.5 / loop_rate  # m/s^2 per loop

class DriveStraightOdom():
    def __init__(self, cmd_vel):
        # Publisher to control the robot's speed
        self.cmd_vel = cmd_vel

        # subscribers to robot data
        rospy.Subscriber("/odom_extra", OdomExtra, self.odom_callback, queue_size=1)
        rospy.Subscriber("/platform_data", PlatformData, self.platform_callback, queue_size=1)
        self.odom_extra = OdomExtra()
        self.platform_data = PlatformData()

        self.r = rospy.Rate(loop_rate)
        self.r.sleep()      #  wait for /odom_extra to populate odometer

    def parse_argv(self, argv):
        self.distance = float(argv[0])  # pick off first arg from supplied list
        return 1            # return number of args consumed

    def print(self):
        print('Drive straight with odometry for {} m'.format(self.distance))

    def run(self):
        move_cmd = Twist()
        odometer_goal = self.odom_extra.odometer + self.distance
        odometer_start = self.odom_extra.odometer
        rospy.loginfo('start odometer: {}, goal: {}'.format(self.odom_extra.odometer, odometer_goal))

        # loop sending motion commands until desired distance attained
        while (not rospy.is_shutdown()):
            if ((self.distance >= 0 and self.odom_extra.odometer >= odometer_goal) or \
                (self.distance < 0 and self.odom_extra.odometer < odometer_goal)):
                break

            if self.distance >= 0:
                move_cmd.linear.x = self.slew_vel(speed)
            else:
                move_cmd.linear.x = self.slew_vel(-speed)

            # rospy.loginfo(move_cmd.linear.x)
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        rospy.loginfo('traveled: {} m'.format(self.odom_extra.odometer - odometer_start))

    def slew_vel(self, to):
        return self.slew(self.platform_data.commandedLinear, to, vel_slew_rate)

    def slew(self, current, to, slew_rate):
        diff = to - current
        if diff > slew_rate:
            return current + slew_rate
        if diff < -slew_rate:
            return current - slew_rate
        return to

    def odom_callback(self, odom_extra_msg):
        self.odom_extra = odom_extra_msg

    def platform_callback(self, platform_data_msg):
        self.platform_data = platform_data_msg

def shutdown():
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def usage():
    print('Usage: drive_straight.py <distance> - drive the specified distance forward or backward')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        usage()
    argv_index = 1

    rospy.init_node('move', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    try:
        m = DriveStraightOdom(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        m.run()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
