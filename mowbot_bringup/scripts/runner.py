#!/usr/bin/env python

import sys
import time
import rospy
import drive_straight_odom
from mowbot_msgs.msg import OdomExtra, PlatformData

odom_extra = OdomExtra()
platform_data = PlatformData()
argv_pos = 1

class drive_straight:
    def __init__(self, distance):
        self.distance = distance
    def print(self):
        print('drive straight for {} m'.format(self.distance))

class rotate_in_place:
    def __init__(self, angle):
        self.angle = angle
    def print(self):
        print('rotate in place {} deg'.format(self.angle))

def drive():
    m = drive_straight(sys.argv[argv_pos+1])
    moves.append(m)
def rotate():
    m = rotate_in_place(sys.argv[argv_pos+1])
    moves.append(m)
def default():
    print ('Error: unrecognized move: {}'.format(sys.argv[argv_pos]))
    sys.exit()

print ('num args: {}'.format(len(sys.argv)))
print('args: {}'.format(str(sys.argv)))
if len(sys.argv) < 3:
    print('error: must provide pairs of args')
    sys.exit()
if len(sys.argv) % 2 != 1:
    print('error: must provide even number of args')
    sys.exit()
moves = []

switcher = {
    'd': drive,
    'r': rotate
    }

def switch(move_rqst):
    # print(sys.argv[argv_pos])
    return switcher.get(sys.argv[argv_pos], default)()

while (argv_pos < len(sys.argv)):
    switch(sys.argv[argv_pos])
    argv_pos += 2

for m in moves:
    m.print()

def shutdown():
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def odom_callback(odom_extra_msg):
    odom_extra = odom_extra_msg

def platform_callback(platform_data_msg):
    platform_data = platform_data_msg

def usage():
    print('Usage: runner.py [commands] - executes the series of move commands provided')
    print('Supported move commands are:')
    print('movo <distance> - drive straight for <distance> meters')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        usage()
    argv_index = 1

    rospy.init_node('runner', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    # Publisher to control the robot's speed
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # subscribers to robot data
    rospy.Subscriber("/odom_extra", OdomExtra, odom_callback, queue_size=1)
    rospy.Subscriber("/platform_data", PlatformData, platform_callback, queue_size=1)
    time.sleep(0.1)     # wait for data to populate

    try:
        m = DriveStraight(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        m.run()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
