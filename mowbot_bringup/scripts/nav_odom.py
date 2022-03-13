#!/usr/bin/env python

from multiprocessing.sharedctypes import Value
import sys
import time
import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2
from MoveParent import MoveParent
from rotate_odom import RotateOdom


err_circle = 0.25    # meters, distance within which we consider goal achieved
dead_zone = pi / 10  # deadzone is +/- this for disablig angular rotation
downramp = 0.5       # downramp is distance at which speed is reduced to slow

class TargetXY():
    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
    def get_xy(self):
        return self.target_x, self.target_y

class NavOdom(MoveParent):
    def __init__(self, cmd_vel):
        super().__init__(cmd_vel)
        self.target_list = []
        self.distance = 0.0
        self.bearing = 0.0

        self.rotating = False
        self.rotator = RotateOdom(cmd_vel)  # instantiate a rotator for use between segments

    def parse_argv(self, argv):
        num_points = 0
        while (len(argv) >= (num_points + 2)):
            try:
                target_x = float(argv[num_points])  # pick off first args from supplied list
                target_y = float(argv[num_points+1])
                t = TargetXY(target_x, target_y)
                self.target_list.append(t)
                num_points += 2
            except ValueError:
                break

        # print('NavOdom parsed {} points'.format(num_points))
        self.current_target = 0     # position in the list of targets
        return num_points           # return number of args consumed

    def print(self):
        print('Navigate with odometry to: ', end='')
        for t in self.target_list:
            x, y = t.get_xy()
            print('[{}, {}], '.format(x, y), end='')
        print('')

    # run is called at the rate until it returns true
    def run(self):
        if self.current_target == len(self.target_list):
            rospy.logwarn('NavOdom.run() called with empty list')
            return True         # done

        if self.rotating:
            if self.rotator.run():
                self.rotating = False       # done rotating, allow driving behavior to recommence
            return False

        target_x, target_y = self.target_list[self.current_target].get_xy()

        if self.once:
            at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)

            rospy.loginfo('Once: [{}, {}] distance: {:.02f} bearing: {:.02f}, at_target: {}'.format(
                target_x, target_y, self.distance, self.bearing, at_target))
            self.once = False

        if self.navigate_target(target_x, target_y):
            # at current_target, set current_target to next in list
            self.current_target += 1
            if self.current_target == len(self.target_list):
                rospy.loginfo('NavOdom done with target list')
                return True         # done
            self.once = True

            # rotate to the heading that points at the next target
            at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)
            bearing_deg = '{:.02f}'.format(self.bearing * (180 / pi))
            rospy.loginfo('NavOdom rotating {} deg to next target'.format(bearing_deg))
            self.rotator.parse_argv([bearing_deg])
            self.rotator.print()
            self.rotating = True
            return False

        return False

    def navigate_target(self, target_x, target_y):
        at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)
        if at_target:
            rospy.loginfo('navigate_target found it is at target')
            return True

        # turn toward target if needed
        if self.bearing < dead_zone and self.bearing > -dead_zone:
            self.move_cmd.angular.z = self.slew_rot(0)
        else:
            if self.bearing > dead_zone:
                self.move_cmd.angular.z = self.slew_rot(self.rot_speed)
            if self.bearing < -dead_zone:
                self.move_cmd.angular.z = self.slew_rot(self.rot_speed)

        # set linear speed, slow down on approach
        if self.distance < downramp:
            speed = self.speed * (self.distance / downramp)
            if speed < self.low_speed:
                speed = self.low_speed
            if speed > self.speed:
                speed = self.speed
            self.move_cmd.linear.x = speed
        else:
            self.move_cmd.linear.x = self.slew_vel(self.speed)

        rospy.logdebug('navigate_target set speed to linear: {:.02f} angular: {:.02f}'.format(self.move_cmd.linear.x, self.move_cmd.angular.z))
        self.cmd_vel.publish(self.move_cmd)

    def target_vector(self, target_x, target_y, distance):
        rospy.logdebug('Entered target_vector({}, {}, {})'.format(target_x, target_y, distance))
        last_distance = distance
        x = target_x - self.odom.pose.pose.position.x
        y = target_y - self.odom.pose.pose.position.y
        distance = sqrt(pow(x, 2) + pow(y, 2))
        heading = self.normalize(self.odom_extra.heading)
        bearing = atan2(y, x) - heading
        bearing_normalized = self.normalize(bearing)
        at_target = self.target_acquired(distance, last_distance)
        rospy.logdebug('target_vector: distance: {:.02f} heading: {:.02f} bearing: {:.02f} normalized bearing: {:.02f}'.format(distance, heading, bearing, bearing_normalized))
        return at_target, distance, bearing_normalized

    def target_acquired(self, distance, last_distance):
        rospy.logdebug('Entered target_acquired({}, {}'.format(distance, last_distance))
        # TODO re-enable distance check: and distance > last_distance):
        if (distance < err_circle): 
            return True
        else:
            return False

    def normalize(self, angle):     # normalize angle to +/- pi
        if angle > pi:
            angle -= 2 * pi
        if angle < -pi:
            angle += 2 * pi
        return angle

def shutdown():
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def usage():
    print('Usage: nav_odom.py <target_x> <target_y> [ more_targets ] - navigate to a list of targets')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        usage()
    argv_index = 1

    rospy.init_node('nav_odom', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    loop_rate = 10       # loop rate
    r = rospy.Rate(loop_rate)

    try:
        m = NavOdom(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        while (not rospy.is_shutdown()):
            if m.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
