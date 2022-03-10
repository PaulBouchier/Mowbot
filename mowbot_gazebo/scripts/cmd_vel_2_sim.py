#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import PlatformData

platform_data = PlatformData()

def cmd_vel_callback(cmd_vel_data):
    gz_cmd_vel.publish(cmd_vel_data)
    global platform_data
    platform_data.commandedLinear = cmd_vel_data.linear.x
    platform_data.commandedAngular = cmd_vel_data.angular.z
    gz_platform_data_pub.publish(platform_data)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_2_sim')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    gz_cmd_vel = rospy.Publisher('/rl500_diff_drive_controller/cmd_vel', Twist, queue_size=1)
    gz_platform_data_pub = rospy.Publisher('/platform_data', PlatformData, queue_size=1)
    rospy.spin()
