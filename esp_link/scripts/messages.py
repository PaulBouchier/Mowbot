import sys
import rospy
from time import sleep, time
from pySerialTransfer import pySerialTransfer as txfer

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

MAX_SPEED = 0.47      # meters/second
BASE_WIDTH = 0.424    # meters, 16.675"

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
        # rospy.logdebug ('Odometry callback got msg length: {}'.format(self.link.bytesRead))
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

        self.pub.publish(self.odom)

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
        self.lastRqstTime = time()
        rospy.Subscriber("/cmd_vel", Twist, self.callback, queue_size=1)

    def callback(self, cmd_vel):
        """ Transform robot m/sec, rad/sec into left/right wheel speeds in m/s """
        vel = cmd_vel.linear.x
        delta_vel = cmd_vel.angular.z * (BASE_WIDTH / 2.0)
        # clamp max wheel speed rqst to +/- MAX_SPEED and ratio vel, delta_vel accordingly
        k = max(abs(vel-delta_vel),abs(vel+delta_vel))
        if (k > MAX_SPEED):
            vel = vel * MAX_SPEED / k
            delta_vel = delta_vel * MAX_SPEED / k
            rospy.logwarn_throttle(1.0, "Requested speed > MAX_SPEED, throttling speed")
        wheel_speeds = [ (vel - delta_vel), (vel + delta_vel) ]
        left_wheel_pct = int((wheel_speeds[0] / MAX_SPEED) * 100)
        right_wheel_pct = int((wheel_speeds[1] / MAX_SPEED) * 100)
        if abs(left_wheel_pct) > 100 or abs(right_wheel_pct) > 100:
            rospy.logerr("requested drive power > 100")
        rospy.logdebug_throttle(
            1.0,
            "cmd_vel (linear/ang): {:.2f}, {:.2f} produced wheel_speeds (left/right): {:.2f} {:.2f}, drive pct: {} {}".format(
            cmd_vel.linear.x, cmd_vel.angular.z, wheel_speeds[0], wheel_speeds[1], left_wheel_pct, right_wheel_pct))
        if ((time() - self.lastRqstTime) < 0.05):
            # rospy.logwarn("Dropped too-soon motors rqst")
            return
        self.post(left_wheel_pct, right_wheel_pct)
        self.lastRqstTime = time()


    def post(self, left_drive_pct, right_drive_pct):
        self.left_drive_pct = left_drive_pct
        self.right_drive_pct = right_drive_pct
        if self.posted:
            rospy.logwarn("TxDriveMotorsRqst request overrun, dropping request")
            return
        # rospy.logdebug ("speed_rqst posted: left: {} right: {}".format(self.left_drive_pct, self.right_drive_pct))
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        send_size = self.link.tx_obj(self.left_drive_pct, val_type_override='b')
        send_size = self.link.tx_obj(self.right_drive_pct, start_pos=send_size, val_type_override='b')
        self.link.send(send_size, 1)
        self.posted = False

