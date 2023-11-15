#!/usr/bin/env python

'''
Monitor RTK-GPS and convert to an Odometry message referenced
to a local 0,0 reference
'''
import sys
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from mowbot_msgs.msg import OdomExtra

zero_x = 0.0
zero_y = 0.0
odom_msg = Odometry()

def utm_callback(utm_msg):
    odom_msg.header = utm_msg.header
    x = utm_msg.pose.position.x - zero_x
    odom_msg.pose.pose.position.x = x
    y = utm_msg.pose.position.y - zero_y
    odom_msg.pose.pose.position.y = y
    odom_pub.publish(odom_msg)
    rospy.logdebug('utm msg published in odom as {:0.3f}, {:0.3f}'.format(x, y))

if __name__ == '__main__':
    rospy.init_node('utm2odom')
    rospy.Subscriber('utm', PoseStamped, utm_callback)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

    
    if (rospy.has_param('~zero_x')):
        zero_x = rospy.get_param('~zero_x')
        rospy.loginfo('Reference x: {}'.format(zero_x))

    if (rospy.has_param('~zero_y')):
        zero_y = rospy.get_param('~zero_y')
        rospy.loginfo('Reference y: {}'.format(zero_y))
    
    rospy.spin()
