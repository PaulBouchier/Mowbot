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
min_radius = 0.2

class DriveArc(MoveParent):
    def __init__(self, cmd_vel):
        super().__init__(cmd_vel)
        self.stopping = False

    def parse_argv(self, argv):
        if len(argv) < 3:
            rospy.logerr('Incorrect number of args provided to DriveArc')
            sys.exit()
        try:
            angle_deg = float(argv[0])  # pick off first arg from supplied list - angle to turn in deg - +ve = CCW
            self.angle = angle_deg * (pi / 180)
            self.radius = float(argv[1])
            if self.radius < min_radius:
                rospy.logerr('DriveArc: radius must be > {}'.format(min_radius))
                raise ValueError
            if argv[2] != 'f' and argv[2] != 'b':
                raise ValueError
            if argv[2] == 'b':
                self.speed = -self.speed
        except ValueError:
            rospy.logerr('Error parsing args in DriveArc')
            sys.exit()

        self.heading_start = self.odom_extra.heading
        self.heading_goal = self.odom_extra.heading + self.angle
        self.crossing2pi = int(self.heading_goal / (2 * pi))                 # can take values -1, 0, 1. 0 means not crossing 2pi
        self.heading_goal -= self.crossing2pi * 2 * pi                       # constrain heading_goal to +/- 2pi
        self.stopping = False

        return 3            # return number of args consumed

    def print(self):
        rospy.loginfo('drive arc {} deg, radius: {:.02f}m, speed: {}'.format(self.angle * (180 / pi), self.radius, self.speed))

    # run is called at the rate until it returns true
    def run(self):
        if self.once:
            rospy.loginfo('start heading: {:.2f}, goal: {:.2f}, crossing2pi: {}'.format(self.heading_start, self.heading_goal, self.crossing2pi))
            self.once = False

        if self.stopping:
            if abs(self.odom.twist.twist.angular.z) < 0.01:     # wait till we've stopped
                self.once = True
                rospy.loginfo('rotated: {} deg to heading {:.2f}'.format((self.odom_extra.heading - self.heading_start) * (180 / pi), self.odom_extra.heading))
                return True
            else:
                return False

        # stop if we've gone past the goal
        if self.crossing2pi == 0 and \
            ((self.angle >= 0 and self.odom_extra.heading >= self.heading_goal) or \
            (self.angle < 0 and self.odom_extra.heading < self.heading_goal)):

            self.move_cmd.angular.z = 0     # exceeded goal, stop immediately
            self.move_cmd.linear.x = 0
            self.cmd_vel.publish(self.move_cmd)
            self.stopping = True        # wait until we've stopped before exiting

            return False

        # +ve angle means turn left. Adjust pi-crossing detection to avoid early exit without movement
        # Angular velocity omega = arc_radius / linear_velocity
        # from theta = radius / distance
        self.move_cmd.linear.x = self.slew_vel(self.speed)
        if self.angle >= 0:
            self.move_cmd.angular.z = abs(self.move_cmd.linear.x) / self.radius  # ramp angular velocity with linear vel
            if ((self.crossing2pi != 0) and (self.odom_extra.heading < (self.heading_start - 0.1))):
                self.crossing2pi = 0                 # robot crossed 2pi, now let the end-of-rotate logic work
        else:
            self.move_cmd.angular.z = -(abs(self.move_cmd.linear.x / self.radius))  # ramp angular velocity with linear vel
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
    print('Usage: drive_arc.py <angle_deg> <radius> < f | b > - rotate the specified number of degrees in an arc, fwd or back')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        usage()
    argv_index = 1

    rospy.init_node('drive_arc', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(loop_rate)

    try:
        m = DriveArc(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        while (not rospy.is_shutdown()):
            if m.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
