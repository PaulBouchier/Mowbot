import sys
import rospy
from time import sleep
from pySerialTransfer import pySerialTransfer as txfer

from nav_msgs.msg import Odometry

class TxPing:
    def __init__(self, link):
        self.link = link
        self.posted = False

    def post(self):
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        ping_type = '\x00'
        send_size = self.link.tx_obj(ping_type)
        self.link.send(send_size, 0)
        self.posted = False

class RxPong:
    def __init__(self, link):
        self.link = link

    def handle_pong(self):
        self.ping_type = b'\x00'
        self.timestamp = 0
        self.timestamp = self.link.rx_obj(obj_type=type(self.timestamp), obj_byte_size=4, list_format='i')
        self.pong_type = self.link.rx_obj(obj_type=type(self.ping_type), obj_byte_size=1, list_format='b')
        rospy.loginfo ("pong type {} timestamp: {}".format(self.pong_type, self.timestamp))

class RxOdometry:
    def __init__(self, link):
        self.link = link
        self.pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odom = Odometry()
        self.sequence = 0
        self.log_rate = 50
        self.log_cnt = 0

    def handle_odometry(self):
        rospy.logdebug ('Odometry callback got msg length: {}'.format(self.link.bytesRead))
        self.odom.header.seq = self.sequence
        self.sequence += 1
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = 'base_link'
        rec_size = 0
        self.odom.pose.pose.position.x = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.odom.pose.pose.position.y = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        heading_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.odom.twist.twist.linear.x = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        self.odom.twist.twist.linear.y = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        linear_speed = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        angular_speed = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        odometer = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        left_speed = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        right_speed = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        left_enc_cnt = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        right_enc_cnt = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']

        self.log_cnt += 1
        if (self.log_cnt == self.log_rate):
            self.log_cnt = 0
            rospy.loginfo("poseX: {:.2f} poseY: {:.2f} heading: {:.2f} speed {:.2f}, odom: {:.2f}".format(
                self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, heading_rad, linear_speed, odometer))


class RxLog:
    def __init__(self, link):
        self.link = link

    def handle_log(self):
        log_length = self.link.bytesRead

        # get the log timestamp
        offset = 0
        timestamp = self.link.rx_obj(obj_type='i', obj_byte_size=4)

        # get the log msg, offset 4 bytes from beginning (after timestamp)
        offset += 4
        log_msg = ''
        for i in range(log_length - offset):
            log_msg += (self.link.rx_obj(obj_type='c', start_pos=i+offset)).decode('utf-8')
        
        # publish the log as a ros log
        timestamp_sec = float(timestamp) / 1000.0
        level = log_msg[0]
        {
            'V': rospy.logdebug,
            'T': rospy.logdebug,
            'I': rospy.loginfo,
            'W': rospy.logwarn,
            'E': rospy.logerr,
            'F': rospy.logfatal,
        }[level](log_msg + " @ {:.3f}".format(timestamp_sec) + "s")

class TxDriveMotorsRqst:
    def __init__(self, link):
        self.link = link
        self.posted = False
        self.left_drive_pct = 0
        self.right_drive_pct = 0

    def post(self, left_drive_pct, right_drive_pct):
        self.left_drive_pct = left_drive_pct
        self.right_drive_pct = right_drive_pct
        rospy.logdebug ("speed_rqst posted: left: {} right: {}".format(self.left_drive_pct, self.right_drive_pct))
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        send_size = self.link.tx_obj(self.left_drive_pct, val_type_override='b')
        send_size = self.link.tx_obj(self.right_drive_pct, start_pos=send_size, val_type_override='b')
        self.link.send(send_size, 1)
        self.posted = False

