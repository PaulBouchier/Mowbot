#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from mowbot_msgs.msg import OdomExtra
from tf.transformations import euler_from_quaternion
from math import radians, copysign, sqrt, pow, pi, asin, atan2


class OdomFromSim():
    def __init__(self):
        rospy.init_node('odom_from_sim')

        self.odom = Odometry
        self.once = None
        self.odom_extra = OdomExtra()

        rospy.Subscriber('/rl500_diff_drive_controller/odom', Odometry, self.odom_cb)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_extra_pub = rospy.Publisher('/odom_extra', OdomExtra, queue_size=1)

        # defaults - updated from odom deltas
        self.last_x = 0
        self.last_y = 0
        self.last_pi_heading = 0
        self.odometer = 0

        # set up to call the odom_extra publisher at 20 Hz
        rospy.Timer(rospy.Duration(0.05), self.pub_odom_extra)


    def pub_odom_extra(self, event):
        if self.once == None:
            return
        if self.once == True:
            self.once = False

            # Initialize variables once odom is available
            self.last_x = self.odom.pose.pose.position.x
            self.last_y = self.odom.pose.pose.position.y
            q = self.odom.pose.pose.orientation
            (r, p, y) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.odom_extra.heading = y
            self.last_pi_heading = y
            return

        self.odom_extra.position = self.odom.pose.pose.position

        # update odometer field
        delta_x = self.odom_extra.position.x - self.last_x
        delta_y = self.odom_extra.position.y - self.last_y
        self.last_x = self.odom_extra.position.x
        self.last_y = self.odom_extra.position.y

        delta_odom = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        if self.odom.twist.twist.linear.x < 0:
            delta_odom = -delta_odom
        self.odom_extra.odometer += delta_odom

        q = self.odom.pose.pose.orientation
        (r, p, y) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        delta_heading = y - self.last_pi_heading
        if delta_heading > pi:
            delta_heading -= 2 * pi     # turning CW
        elif delta_heading < -pi:
            delta_heading += 2 * pi     # turning CCW
        
        self.last_pi_heading = y
        self.odom_extra.heading += delta_heading
        self.odom_extra.heading -= int(self.odom_extra.heading / (2 * pi)) * 2 * pi       # constrain heading to +/- 2pi

        # print('odom: {:.2f} heading: {:.2f} delta_heading {:.2f} y: {:.2f}'.format(
        #     self.odom_extra.odometer, self.odom_extra.heading, delta_heading, y))

        self.odom_extra_pub.publish(self.odom_extra)


    def odom_cb(self, odometry):
        self.odom_pub.publish(odometry)
        self.odom = odometry
        if self.once == None:
            self.once = True

if __name__ == '__main__':
    try:
        OdomFromSim()
    except Exception as e:
        print(e)
        rospy.loginfo("odom_from_sim node terminated.")

    rospy.loginfo('OdomFromSim is running')
    rospy.spin()
