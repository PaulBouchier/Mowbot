#!/usr/bin/env python

from multiprocessing.sharedctypes import Value
import sys
import time
import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import NavDebug
import tf
from math import sqrt, pow, pi, atan2
from MoveParent import MoveParent

err_circle = 1.0    # meters, distance within which we consider goal achieved
dead_zone = pi / 40  # deadzone is +/- this for disablig angular rotation
downramp = 0.75       # downramp is distance at which speed is reduced to slow

class TargetXY():
    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
    def get_xy(self):
        return self.target_x, self.target_y

class NavOdom(MoveParent):
    def __init__(self, cmd_vel):
        super().__init__(cmd_vel)
        self.distance = 0.0
        self.bearing = 0.0
        self.stopping = False

        self.nav_debug = NavDebug()
        self.nav_debug_pub = rospy.Publisher('/nav_debug', NavDebug, queue_size=1)
        time.sleep(0.2)
        # set up to call the nav_debug publisher at 10 Hz
        rospy.Timer(rospy.Duration(0.1), self.pub_nav_debug)

    def parse_argv(self, argv):
        num_points = 0
        self.target_list = []
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
            target_x, target_y = self.target_list[self.current_target].get_xy()
            self.distance = 0
            self.once = True

            # rotate to the heading that points at the next target
            at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)
            bearing_deg = '{:.02f}'.format(self.bearing * (180 / pi))
            return False

        return False

    def navigate_target(self, target_x, target_y):
        at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)
        if self.stopping:
            if self.odom.twist.twist.linear.x < 0.01 and self.odom.twist.twist.angular.z < 0.01:
                self.stopping = False
                return True
            else:
                return False        # wait for robot to stop
        if at_target:
            rospy.loginfo('navigate_target found it is at target, stopping')
            self.cmd_vel.publish(Twist())       # send stop command
            self.stopping = True
            return False

        # turn toward target if needed. Don't turn if within the error circle
        if self.bearing < dead_zone and self.bearing > -dead_zone:
            self.move_cmd.angular.z = self.slew_rot(0)
        else:
            if self.bearing > dead_zone and self.distance > err_circle:
                self.move_cmd.angular.z = self.slew_rot(self.rot_speed)
            if self.bearing < -dead_zone and self.distance > err_circle:
                self.move_cmd.angular.z = self.slew_rot(-self.rot_speed)

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
        x_dist = target_x - self.odom.pose.pose.position.x
        y_dist = target_y - self.odom.pose.pose.position.y
        distance = sqrt(pow(x_dist, 2) + pow(y_dist, 2))
        heading = self.normalize(self.odom_extra.heading)
        # from dpa page: target_angle = (90 - (atan2(yd,xd)*(180/PI))) - (heading*(180/PI));
        bearing = atan2(y_dist, x_dist) - heading
        bearing_normalized = self.normalize(bearing)
        at_target = self.target_acquired(distance, last_distance)

        # publish debug data
        rospy.logdebug('target_vector: distance: {:.02f} heading: {:.02f} bearing: {:.02f} normalized bearing: {:.02f}'.format(distance, heading, bearing, bearing_normalized))
        self.nav_debug.target_x = target_x
        self.nav_debug.target_y = target_y
        self.nav_debug.at_target = at_target
        self.nav_debug.x_distance = x_dist
        self.nav_debug.y_distance = y_dist
        self.nav_debug.distance = distance
        self.nav_debug.bearing = bearing
        self.nav_debug.bearing_normalized = bearing_normalized

        return at_target, distance, bearing_normalized

    def target_acquired(self, distance, last_distance):
        rospy.logdebug('Entered target_acquired({}, {}'.format(distance, last_distance))
        if (distance < err_circle and distance > last_distance): 
            return True
        else:
            return False

    def normalize(self, angle):     # normalize angle to +/- pi
        if angle > pi:
            angle -= 2 * pi
        if angle < -pi:
            angle += 2 * pi
        return angle

    def pub_nav_debug(self, event):
        self.nav_debug_pub.publish(self.nav_debug)


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
