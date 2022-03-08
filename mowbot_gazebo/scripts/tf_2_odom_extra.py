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

        odom_extra_pub = rospy.Publisher('/odom_extra', OdomExtra, queue_size=1)
        self.odom_extra = OdomExtra()

        # verify that we're getting wheel transforms
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('battery_box_link', '/left_wheel_link', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between battery_box_link and left_wheel_link")
            rospy.signal_shutdown("tf Exception") 

        # set up to call the odom_extra publisher at 20 Hz
        rospy.Timer(rospy.Duration(0.2), self.pub_odom_extra)

        rospy.spin()

    def pub_odom_extra(self, event):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform('battery_box_link', 'left_wheel_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        q = Quaternion(*rot)
        #(r, p, y) = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        (r, p, y) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        print('r: {} p: {} y: {}'.format(r, p, y))
    
if __name__ == '__main__':
    try:
        Tf2OdomExtra()
    except Exception as e:
        print(e)
        rospy.loginfo("TF to OdomExtra node terminated.")
