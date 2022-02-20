import sys
import rospy
from pySerialTransfer import pySerialTransfer as txfer

sys.path.append('.')
import config

class TxPing:
    def __init__(self):
        self.posted = False

    def post(self):
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        ping_type = '\x00'
        send_size = config.link.tx_obj(ping_type)
        config.link.send(send_size, 0)
        self.posted = False

class RxPong:
    def handle_pong(self):
        self.ping_type = b'\x00'
        self.timestamp = 0
        self.timestamp = config.link.rx_obj(obj_type=type(self.timestamp), obj_byte_size=4, list_format='i')
        self.pong_type = config.link.rx_obj(obj_type=type(self.ping_type), obj_byte_size=1, list_format='b')
        rospy.loginfo ("pong type {} timestamp: {}".format(self.pong_type, self.timestamp))

class RxOdometry:
    def handle_odometry(self):
        print ('Odometry callback got msg')

class RxLog:
    def handle_log(self):
        log_length = config.link.bytesRead
        print (log_length)

        # get the log timestamp
        offset = 0
        timestamp = config.link.rx_obj(obj_type='i', obj_byte_size=4)

        # get the log msg, offset 4 bytes from beginning (after timestamp)
        offset += 4
        log_msg = ''
        for i in range(log_length - offset):
            log_msg += (config.link.rx_obj(obj_type='c', start_pos=i+offset)).decode('utf-8')
        
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
    def __init__(self):
        self.posted = False
        self.left_drive_pct = 0
        self.right_drive_pct = 0

    def post(self, left_drive_pct, right_drive_pct):
        self.left_drive_pct = left_drive_pct
        self.right_drive_pct = right_drive_pct
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        send_size = config.link.tx_obj(self.left_drive_pct)
        send_size += config.link.tx_obj(self.left_drive_pct, start_pos=send_size)
        config.link.send(send_size, 1)
        self.posted = False

