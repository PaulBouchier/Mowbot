#!/usr/bin/env python
import rospy
from nmea_msgs.msg import Sentence

def talker():
    pub = rospy.Publisher('nmea', Sentence, queue_size=1)
    rospy.init_node('nmea_gga', anonymous=False)
    rate = rospy.Rate(1) # 1 hz
    nmea_gga = "$GPGGA,024535.313,3309.480,N,09656.208,W,1,12,1.0,0.0,M,0.0,M,,*7B"
    nmea_sentence = Sentence()
    nmea_sentence.sentence = nmea_gga
    while not rospy.is_shutdown():
        pub.publish(nmea_sentence)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
