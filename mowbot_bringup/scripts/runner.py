#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist
import drive_straight_odom, stop, rotate_odom
from mowbot_msgs.msg import OdomExtra, PlatformData

loop_rate = 10

def shutdown():
    global cmd_vel
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def usage():
    print('Usage: runner.py [commands] - executes the series of move commands provided')
    print('Supported move commands are:')
    print('movo <distance> [speed] - drive straight for <distance> meters')
    print('roto <angle> [speed] - rotate <angle> degrees, +ve is CCW')
    print('stop - ramp linear and rotational speed down to 0')
    sys.exit()

# ------ Switch statement ------
def movo_case(argv):
    m = drive_straight_odom.DriveStraightOdom(cmd_vel)
    return m

def roto_case(argv):
    m = rotate_odom.RotateOdom(cmd_vel)
    return m

def stop_case(argv):
    m = stop.Stop(cmd_vel)
    return m

def default_case(argv):
    print ('Error: unrecognized move: {}'.format(argv[0]))
    sys.exit()

switcher = {
    'movo': movo_case,
    'roto': roto_case,
    'stop': stop_case
    }

def switch(move_rqst):
    return switcher.get(sys.argv[argv_index], default_case)(sys.argv[argv_index:])

# ------ End switch statement ------

if __name__ == '__main__':
    if len(sys.argv) < 2:
        usage()

    rospy.init_node('runner', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    # Publisher to control the robot's speed
    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(loop_rate)

    # argv parser creates objects to execute each move primitive and initializes each with provided args
    argv_index = 1
    moves = []
    while (argv_index < len(sys.argv)):
        moves.append(switch(sys.argv[argv_index]))
        argv_index += 1     # advanced past the command to its args or next command
        argv_index += moves[-1].parse_argv(sys.argv[argv_index:])

    print('mowbot is about to execute the following moves:')
    for m in moves:
        m.print()
    # reply = input('Continue? [y/n] or <Enter>')
    # if reply != 'y' and reply != '':
    #     sys.exit()

    try:
        for m in moves:
            while (not m.run()):
                r.sleep()
                if rospy.is_shutdown():
                    break

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
