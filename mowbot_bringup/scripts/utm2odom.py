#!/usr/bin/env python

'''
Monitor RTK-GPS and convert to an Odometry message referenced
to a local 0,0 reference
'''
import sys
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from mowbot_msgs.msg import OdomExtra
from mavros_msgs.msg import RTCM
from rtcm_msgs.msg import Message
from tf2_ros import TransformBroadcaster

zero_x = 0.0
zero_y = 0.0
odom_msg = Odometry()
odom_extra_msg = OdomExtra()
rtcm_msg = Message()
br = TransformBroadcaster()
t = TransformStamped()

def rtcm_callback(msg):
    rtcm_msg.header = msg.header
    rospy.logdebug("Published rtcm msg")
    rtcm_msg.message = msg.data
    ntrip_pub.publish(rtcm_msg)

def odom_extra_callback(msg):
    odom_extra_msg = msg

def utm_callback(utm_msg):
    odom_msg.header = utm_msg.header
    x = utm_msg.pose.position.x - zero_x
    odom_msg.pose.pose.position.x = x
    y = utm_msg.pose.position.y - zero_y
    odom_msg.pose.pose.position.y = y
    odom_pub.publish(odom_msg)
    rospy.logdebug('utm msg published in odom as {:0.3f}, {:0.3f}'.format(x, y))

    t.header.stamp = utm_msg.header.stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"


if __name__ == '__main__':
    rospy.init_node('utm2odom')
    rospy.Subscriber('utm', PoseStamped, utm_callback)
    rospy.Subscriber('/odom_extra', OdomExtra, odom_extra_callback)
    rospy.Subscriber('/ntrip_client/rtcm', RTCM, rtcm_callback)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    ntrip_pub = rospy.Publisher('/rtcm', Message, queue_size=1)

    
    if (rospy.has_param('~zero_x')):
        zero_x = rospy.get_param('~zero_x')
        rospy.loginfo('Reference x: {}'.format(zero_x))

    if (rospy.has_param('~zero_y')):
        zero_y = rospy.get_param('~zero_y')
        rospy.loginfo('Reference y: {}'.format(zero_y))
    
    rospy.spin()
