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
vel_slew_rate = 0.5 / loop_rate  # m/s^2 per loop
rot_slew_rate = 1.0 / loop_rate  # rad/s^2 per loop

class Stop():
    def __init__(self, cmd_vel):
        self.once = True
        self.cmd_vel = cmd_vel      # cmd_vel publisher

        # subscribers to robot data
        rospy.Subscriber("/odom_extra", OdomExtra, self.odom_callback, queue_size=1)
        rospy.Subscriber("/platform_data", PlatformData, self.platform_callback, queue_size=1)
        self.odom_extra = OdomExtra()
        self.platform_data = PlatformData()

        self.move_cmd = Twist()
        time.sleep(0.1)

    def parse_argv(self, argv):
        return 0            # return number of args consumed

    def print(self):
        print('Stop')

    def run(self):
        if self.once:
            start_odom = self.odom_extra.odometer
            rospy.loginfo('Stop: start odometer: {}, heading: {}'.format(self.odom_extra.odometer, self.odom_extra.heading))

        # loop sending stop commands until both linear & angular request stopped
        if (abs(self.platform_data.commandedLinear) < 0.01 and abs(self.platform_data.commandedAngular) < 0.01):
            return True

        self.move_cmd.linear.x = self.slew_vel(0)
        self.move_cmd.angular.z = self.slew_rot(0)

        # rospy.loginfo(self.move_cmd.linear.x)
        self.cmd_vel.publish(self.move_cmd)
        self.r.sleep()

        self.cmd_vel.publish(Twist())   # stop the robot
        return False

    def slew_vel(self, to):
        return self.slew(self.platform_data.commandedLinear, 0.0, vel_slew_rate)

    def slew_rot(self, to):
        return self.slew(self.platform_data.commandedAngular, 0, rot_slew_rate)

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
    print('Usage: stop.py - ramp down linear & rotational speed')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) != 1:
        usage()
    argv_index = 1

    rospy.init_node('stop', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    # Publisher to control the robot's speed
    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(loop_rate)

    try:
        m = Stop(cmd_vel)
        m.print()
        while (not rospy.is_shutdown()):
            if m.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
