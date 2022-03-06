#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2

loop_rate = 10       # loop rate
speed = 0.25    # driving speed, fwd or back
vel_slew_rate = 0.3 / loop_rate  # m/s^2 per loop

odom_extra = OdomExtra()
platform_data = PlatformData()

class DriveStraight():
    def __init__(self, cmd_vel):
        self.cmd_vel = cmd_vel      # cmd_vel publisher
        self.r = rospy.Rate(loop_rate)
        self.r.sleep()      #  wait for /odom_extra to populate odometer

    def parse_argv(self, argv):
        self.distance = float(argv[0])  # pick off first arg from supplied list
        return 1

    def print(self):
        print('driving straight with odometry for {} m'.format(self.distance))

    def run(self):
        self.move_cmd = Twist()
        self.odometer_goal = odom_extra.odometer + self.distance
        self.odometer_start = odom_extra.odometer
        rospy.loginfo('start odometer: {}, goal: {}'.format(odom_extra.odometer, self.odometer_goal))

        # loop sending motion commands until desired distance attained
        while (not rospy.is_shutdown()):
            if ((self.distance >= 0 and odom_extra.odometer >= self.odometer_goal) or \
                (self.distance < 0 and odom_extra.odometer < self.odometer_goal)):
                break

            if self.distance >= 0:
                self.move_cmd.linear.x = self.slew_vel(speed)
            else:
                self.move_cmd.linear.x = self.slew_vel(-speed)

            # rospy.loginfo(self.move_cmd.linear.x)
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

        self.cmd_vel.publish(Twist())   # stop the robot
        rospy.sleep(1)
        rospy.loginfo('traveled: {} m'.format(odom_extra.odometer - self.odometer_start))

    def slew_vel(self, to):
        last_speed_cmd = (platform_data.leftMps + platform_data.rightMps) / 2.0
        return self.slew(last_speed_cmd, to, vel_slew_rate)

    def slew(self, current, to, slew_rate):
        diff = to - current
        if diff > slew_rate:
            return current + slew_rate
        if diff < -slew_rate:
            return current - slew_rate
        return to

def shutdown():
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def odom_callback(odom_extra_msg):
    odom_extra = odom_extra_msg

def platform_callback(platform_data_msg):
    platform_data = platform_data_msg

def usage():
    print('Usage: drive_straight.py <distance> - drive the specified distance forward or backward')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        usage()
    argv_index = 1

    rospy.init_node('move', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    # Publisher to control the robot's speed
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # subscribers to robot data
    rospy.Subscriber("/odom_extra", OdomExtra, odom_callback, queue_size=1)
    rospy.Subscriber("/platform_data", PlatformData, platform_callback, queue_size=1)
    time.sleep(0.1)     # wait for data to populate

    try:
        m = DriveStraight(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        m.run()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
