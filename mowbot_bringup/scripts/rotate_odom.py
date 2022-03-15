#!/usr/bin/env python

import sys
import time
from numpy import true_divide
import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2
from MoveParent import MoveParent

loop_rate = 10       # loop rate
target_close_angle = 0.3    # slow down when this close
angle_correction_sim = 0.970

class RotateOdom(MoveParent):
    def __init__(self, cmd_vel):
        super().__init__(cmd_vel)
        self.rot_stopping = False

    def parse_argv(self, argv):
        angle_deg = float(argv[0])  # pick off first arg from supplied list - angle to turn in deg - +ve = CCW
        self.angle = angle_deg * (pi / 180) * angle_correction_sim

        self.heading_start = self.odom_extra.heading
        self.heading_goal = self.odom_extra.heading + self.angle
        self.crossing2pi = int(self.heading_goal / (2 * pi))                 # can take values -1, 0, 1. 0 means not crossing 2pi
        self.heading_goal -= self.crossing2pi * 2 * pi                       # constrain heading_goal to +/- 2pi
        self.rot_stopping = False

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

    # run is called at the rate until it returns true
    def run(self):
        if self.once:
            rospy.loginfo('start heading: {:.2f}, goal: {:.2f}, crossing2pi: {}'.format(self.heading_start, self.heading_goal, self.crossing2pi))
            self.once = False
            self.rot_speed = self.full_rot_speed

        if self.rot_stopping:
            if abs(self.odom.twist.twist.angular.z) < 0.01:     # wait till we've stopped
                self.once = True
                rospy.loginfo('rotated: {} deg to heading {:.2f}'.format((self.odom_extra.heading - self.heading_start) * (180 / pi), self.odom_extra.heading))
                return True
            else:
                return False

        # slow the rotation speed if we're getting close
        if self.crossing2pi == 0:
            if ((self.angle >= 0 and self.odom_extra.heading > (self.heading_goal - target_close_angle)) or
                (self.angle < 0 and self.odom_extra.heading < (self.heading_goal + target_close_angle))):
                self.rot_speed = self.low_rot_speed

        # stop if we've gone past the goal
        if self.crossing2pi == 0 and \
            ((self.angle >= 0 and self.odom_extra.heading >= self.heading_goal) or \
            (self.angle < 0 and self.odom_extra.heading < self.heading_goal)):

            self.move_cmd.angular.z = 0     # exceeded goal, stop immediately
            self.cmd_vel.publish(self.move_cmd)
            self.rot_stopping = True        # wait until we've stopped before exiting

            return False

        # +ve angle means turn left. Adjust pi-crossng detection to avoid early exit without movement
        if self.angle >= 0:
            self.move_cmd.angular.z = self.slew_rot(self.rot_speed)
            if ((self.crossing2pi != 0) and (self.odom_extra.heading < (self.heading_start - 0.1))):
                self.crossing2pi = 0                 # robot crossed 2pi, now let the end-of-rotate logic work
        else:
            self.move_cmd.angular.z = self.slew_rot(-self.rot_speed)
            if ((self.crossing2pi != 0) and (self.odom_extra.heading > (self.heading_start + 0.1))):
                self.crossing2pi = 0                 # robot crossed 2pi, now let the end-of-rotate logic work

        # rospy.loginfo(self.odom_extra.heading)
        self.cmd_vel.publish(self.move_cmd)

        return False

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
