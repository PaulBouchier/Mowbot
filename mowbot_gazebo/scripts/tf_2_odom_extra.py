#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Quaternion
from mowbot_msgs.msg import OdomExtra
import tf
from tf.transformations import euler_from_quaternion
from math import radians, copysign, sqrt, pow, pi, asin, atan2


class Tf2OdomExtra():
    def __init__(self):
        rospy.init_node('tf_2_odom_extra', anonymous=False)

        self.odom_extra_pub = rospy.Publisher('/odom_extra', OdomExtra, queue_size=1)
        self.odom_extra = OdomExtra()

        # verify that we're getting wheel transforms
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between battery_box_link and left_wheel_link")
            rospy.signal_shutdown("tf Exception") 

        # defaults - updated from tf deltas
        self.last_x = 0
        self.last_y = 0
        self.last_pi_heading = 0
        self.odometer = 0

        # Get the current transform between the odom and base frames
        try:
            (trans, rotation)  = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception in __init__()")
            return

        position = Point(*trans)
        self.last_x = position.x
        self.last_y = position.y

        q = Quaternion(*rotation)
        (r, p, y) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_extra.heading = y
        self.last_pi_heading = y

        # set up to call the odom_extra publisher at 20 Hz
        rospy.Timer(rospy.Duration(0.2), self.pub_odom_extra)

        rospy.spin()

    def pub_odom_extra(self, event):
        # Get the current transform between the odom and base frames
        try:
            (trans, rotation)  = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception in pub_odom_extra")
            return

        self.odom_extra.position = Point(*trans)

        # update odometer field
        delta_x = self.odom_extra.position.x - self.last_x
        delta_y = self.odom_extra.position.y - self.last_y
        self.last_x = self.odom_extra.position.x
        self.last_y = self.odom_extra.position.y
        delta_odom = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        self.odom_extra.odometer += delta_odom

        q = Quaternion(*rotation)
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
    
if __name__ == '__main__':
    try:
        Tf2OdomExtra()
    except Exception as e:
        print(e)
        rospy.loginfo("TF to OdomExtra node terminated.")
