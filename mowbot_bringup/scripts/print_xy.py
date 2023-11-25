#!/usr/bin/env python

from nav_msgs.msg import Odometry
from mowbot_msgs.msg import OdomExtra, PlatformData
import rospy
from math import pi

global odom, odom_extra
print_period = 5
odom = Odometry()
odom_extra = OdomExtra()

def odom_callback(odom_msg):
    odom.pose.pose.position.x = odom_msg.pose.pose.position.x
    odom.pose.pose.position.y = odom_msg.pose.pose.position.y

def odom_extra_callback(odom_extra_msg):
    odom_extra.heading = odom_extra_msg.heading


if __name__ == '__main__':
    rospy.init_node('move', anonymous=False)
    rospy.Subscriber("/odom_extra", OdomExtra, odom_extra_callback, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)


    while(not rospy.is_shutdown()):
        rospy.sleep(3)
        heading = odom_extra.heading
        rospy.loginfo('X: {:.2f} Y: {:.2f} heading: {:.2f} {:.2f}'.format(odom.pose.pose.position.x, odom.pose.pose.position.y, heading, ((heading/(2*pi))*360)))
