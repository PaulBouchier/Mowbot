#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2
from MoveParent import MoveParent

loop_rate = 10       # loop rate

class Stop(MoveParent):
    def __init__(self, cmd_vel):
        super().__init__(cmd_vel)
        self.pause = 0            # pause after stop, in seconds
        self.end_pause_time = rospy.Time.now() # time when pause ends

    def parse_argv(self, argv):
        # check whether a pause argument was supplied
        if (len(argv) > 0):
            try:
                pause_arg = float(argv[0])
                self.pause = pause_arg
                self.end_pause_time = rospy.Time.now() + rospy.Duration(self.pause)  # time when pause ends
                print('Using supplied pause {}'.format(self.pause))
                return 1
            except ValueError:
                return 0
        return 0            # return number of args consumed

    def print(self):
        print('Stop and pause for {} seconds'.format(self.pause))

    def run(self):
        if self.once:
            start_odom = self.odom_extra.odometer
            rospy.loginfo('Stop: start odometer: {}, heading: {}'.format(self.odom_extra.odometer, self.odom_extra.heading))
            self.once = False

        # loop sending stop commands until both linear & angular request stopped
        if (abs(self.platform_data.commandedLinear) < 0.01 and abs(self.platform_data.commandedAngular) < 0.01):
            self.cmd_vel.publish(Twist())   # stop the robot
            if rospy.Time.now() > self.end_pause_time:
                rospy.loginfo('Stop: paused for {} seconds'.format(self.pause))
                return True
            else:
                return False

        self.move_cmd.linear.x = self.slew_vel(0)
        self.move_cmd.angular.z = self.slew_rot(0)

        # rospy.loginfo(self.move_cmd.linear.x)
        self.cmd_vel.publish(self.move_cmd)

        return False

def shutdown():
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def usage():
    print('Usage: stop.py [pause] - ramp down linear & rotational speed & pause if pause is not zero')
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
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        while (not rospy.is_shutdown()):
            if m.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
