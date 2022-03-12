#!/usr/bin/env python

import sys
import time
from numpy import true_divide
import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2

loop_rate = 10       # loop rate
rot_speed_default = 0.5    # rotating speed, rad/s
rot_slew_rate = 0.5 / loop_rate  # rad/s^2 per loop

class RotateOdom():
    def __init__(self, cmd_vel):
        self.once = True
        self.rot_speed = rot_speed_default

        # Publisher to control the robot's speed
        self.cmd_vel = cmd_vel

        # subscribers to robot data
        self.odom_extra = OdomExtra()
        self.platform_data = PlatformData()
        rospy.Subscriber("/odom_extra", OdomExtra, self.odom_callback, queue_size=1)
        rospy.Subscriber("/platform_data", PlatformData, self.platform_callback, queue_size=1)

        self.move_cmd = Twist()
        time.sleep(0.1)

    def parse_argv(self, argv):
        angle_deg = float(argv[0])  # pick off first arg from supplied list - angle to turn in deg - +ve = CCW
        self.angle = angle_deg * (pi / 180)

        # Reduce requested angle to allow for overshoot
        if self.angle > 0.5:
            self.angle -= 0.3
        if self.angle < -0.5:
            self.angle += 0.3
        
        self.heading_goal = self.odom_extra.heading + self.angle
        self.crossing2pi = int(self.heading_goal / (2 * pi))                 # can take values -1, 0, 1. 0 means not crossing 2pi
        self.heading_goal -= self.crossing2pi * 2 * pi                       # constrain heading_goal to +/- 2pi
        self.heading_start = self.odom_extra.heading

        # check whether a rot_speed argument was supplied
        if len(argv) > 1:
            try:
                rot_speed_arg = float(argv[1])
                self.rot_speed = rot_speed_arg
                print('Using supplied rot_speed {}'.format(self.rot_speed))
                return 2
            except ValueError:
                return 1
        return 1            # return number of args consumed

    def print(self):
        print('rotate {} deg'.format(self.angle * 180 / pi))

    def run(self):
        if self.once:
            rospy.loginfo('start heading: {:.2f}, goal: {:.2f}, crossing2pi: {}'.format(self.heading_start, self.heading_goal, self.crossing2pi))
            self.once = False

        # loop sending motion commands until desired distance attained
        while (not rospy.is_shutdown()):
            if self.crossing2pi == 0 and \
                ((self.angle >= 0 and self.odom_extra.heading >= self.heading_goal) or \
                (self.angle < 0 and self.odom_extra.heading < self.heading_goal)):
                rospy.loginfo('rotated: {} deg to heading {:.2f}'.format((self.odom_extra.heading - self.heading_start) * (180 / pi), self.odom_extra.heading))
                return True

            # +ve angle means turn left
            if self.angle >= 0:
                self.move_cmd.angular.z = self.slew_rot(self.rot_speed)
                if ((self.crossing2pi != 0) and (self.odom_extra.heading < self.heading_start)):
                    self.crossing2pi = 0                 # robot crossed 2pi, now let the end-of-rotate logic work
            else:
                self.move_cmd.angular.z = self.slew_rot(-self.rot_speed)
                if ((self.crossing2pi != 0) and (self.odom_extra.heading > self.heading_start)):
                    self.crossing2pi = 0                 # robot crossed 2pi, now let the end-of-rotate logic work

            # rospy.loginfo(self.odom_extra.heading)
            self.cmd_vel.publish(self.move_cmd)
            return False

    def slew_rot(self, to):
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
    print('Usage: drive_straight.py <distance> [rot_speed] - drive the specified distance forward or backward')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        usage()
    argv_index = 1

    rospy.init_node('move', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(loop_rate)

    try:
        m = RotateOdom(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        while (not rospy.is_shutdown()):
            if m.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
