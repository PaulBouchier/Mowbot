#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


gz_cmd_vel = 0

def cmd_vel_callback(cmd_vel_data):
    gz_cmd_vel.publish(cmd_vel_data)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_2_gz_sim')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    gz_cmd_vel = rospy.Publisher('/rl500_diff_drive_controller/cmd_vel', Twist, queue_size=1)
    rospy.spin()
