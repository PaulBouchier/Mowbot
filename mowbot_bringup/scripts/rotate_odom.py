#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2

loop_rate = 10       # loop rate
rot_speed = 0.35    # rotating speed, rad/s
rot_slew_rate = 0.5 / loop_rate  # rad/s^2 per loop

class RotateOdom():
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
        angle_deg = float(argv[0])  # pick off first arg from supplied list - angle to turn in deg - +ve = CCW
        self.angle = angle_deg * (pi / 180)
        return 1            # return number of args consumed

    def print(self):
        print('rotate {} deg'.format(self.angle * 180 / pi))

    def run(self):
        move_cmd = Twist()
        heading_goal = self.odom_extra.heading + self.angle
        heading_goal -= (int(heading_goal / (2 * pi)) * 2 * pi)     # constrain to +/- 2pi
        heading_start = self.odom_extra.heading
        rospy.loginfo('start heading: {}, goal: {}'.format(heading_start, heading_goal))

        # loop sending motion commands until desired distance attained
        while (not rospy.is_shutdown()):
            # FIXME - this is wrong
            if ((self.angle >= 0 and self.odom_extra.heading >= heading_goal) or \
                (self.angle < 0 and self.odom_extra.heading < heading_goal)):
                break

            # +ve angle means turn left
            if self.angle >= 0:
                move_cmd.angular.z = self.slew_vel(rot_speed)
            else:
                move_cmd.angular.z = self.slew_vel(-rot_speed)

            rospy.loginfo(move_cmd.angular.z)
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        rospy.loginfo('rotated: {} deg'.format((self.odom_extra.heading - heading_start) * (180 / pi)))

    def slew_vel(self, to):
        return self.slew(self.platform_data.commandedAngular, to, rot_slew_rate)

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
        m = RotateOdom(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        m.run()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
