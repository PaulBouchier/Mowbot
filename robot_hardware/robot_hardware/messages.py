import rclpy
from rclpy.node import Node

import math
import sys
from time import time
from math import sin, cos, pi
from pySerialTransfer import pySerialTransfer as txfer

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion
from robot_interfaces.msg import OdomExtra, PlatformData
from tf2_ros import TransformBroadcaster

# Tx Packet IDs
pktIdPing = 0
pktIdDriveMotorsRqst = 1
pktIdLogLevel = 2
pktIdReboot = 3
pktIdBITMode = 4
pktIdClearOdom = 5

MAX_SPEED = 0.47      # meters/second
BASE_WIDTH = 0.424    # meters, 16.675"
WHEEL_RADIUS = 0.127  # meters

'''
class RxLog:
    def __init__(self, link):
        self.link = link
        self.last_seq = 0

    def handle_log(self):
        log_length = self.link.bytesRead

        # get the log sequence # & timestamp
        offset = 0
        seq = self.link.rx_obj(obj_type='i', start_pos=offset, obj_byte_size=4)
        if seq != (self.last_seq +1):
            rospy.logerr('RxLog detected dropped log msg; sequence #: {}, expected {}'.format(seq, self.last_seq+1))
        self.last_seq = seq
        offset += 4

        timestamp = self.link.rx_obj(obj_type='i', start_pos=offset, obj_byte_size=4)
        offset += 4

        # get the log msg, offset 4 bytes from beginning (after timestamp)
        log_msg = ''
        for i in range(log_length - offset):
            log_msg += (self.link.rx_obj(obj_type='c', start_pos=offset)).decode('utf-8')
            offset += 1
        
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

class RxOdometry:
    def __init__(self, link):
        self.link = link

        # configure odometry publishing
        self.pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odom = Odometry(header=rospy.Header(frame_id="odom"), 
                        child_frame_id='base_link')
        # Note: the following covariance matrices are made up out of thin air
        ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 1e6, 0, 1e3]
        ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e3]
        self.odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom.twist.covariance = ODOM_TWIST_COVARIANCE

        # configure tf publishing
        self.tf_frame = rospy.get_param('~tf_frame', "odom")
        if self.tf_frame != "":
            self.odomBroadcaster = TransformBroadcaster()
            rospy.loginfo('Publishing tf frame: %s' % self.tf_frame)

        # configure joint_state publishing, consumed by joint_state_publisher
        self.joint_pub = rospy.Publisher('wheel_joints_state', JointState, queue_size=1)
        self.joint_state = JointState(name=['left_wheel_joint', 'right_wheel_joint'])

        # configure odom_extra publishing
        self.odom_extra_pub = rospy.Publisher('odom_extra', OdomExtra, queue_size=1)
        self.odom_extra = OdomExtra()
        self.last_esp_seq = 0

        self.log_rate = 1000    # print a log message every log_rate OdometryMsg's
        self.log_cnt = 0

    def normalize(self, angle):     # normalize angle to +/- pi
        if angle > 2 * pi:
            angle -= 2 * pi
        if angle < 0:
            angle += 2 * pi
        return angle

    def handle_odometry(self):
        # rospy.logdebug ('Odometry callback got msg length: {}'.format(self.link.bytesRead))
        self.odom.header.stamp = rospy.Time.now()
        rec_size = 0

        # populate odom from OdometryMsg
        # member seq
        sequence = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        self.odom.header.seq = sequence
        # member espTimestamp
        espTimestamp = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        # member poseX_m
        self.odom.pose.pose.position.x = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member poseY_m
        self.odom.pose.pose.position.y = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member heading_rad
        heading_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)

        # fix up heading, which is almost 180 degrees off coming from the sensor
        heading_rad = heading_rad - 4.7
        heading_rad = self.normalize(heading_rad)

        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member odom_heading_rad
        odom_heading_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member speedX_mps
        self.odom.twist.twist.linear.x = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member speedY_mps
        self.odom.twist.twist.linear.y = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member linear_speed_mps
        linear_speed = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member angular_speed_rps
        self.odom.twist.twist.angular.z = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member odometer_m
        odometer = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member leftSpeed
        left_speed = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member rightSpeed
        right_speed = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member leftEncoderCount
        left_enc_cnt = self.link.rx_obj(obj_type='i', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['i']
        # member rightEncoderCount
        right_enc_cnt = self.link.rx_obj(obj_type='i', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['i']
        #member leftWheelAngleRad
        left_wheel_angle_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        #member rightWheelAngleRad
        right_wheel_angle_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member imuCalStatus
        IMUCalStatus = self.link.rx_obj(obj_type='i', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['i']

        # fix up heading

        # populate pose heading quaternion
        quaternion = Quaternion()
        quaternion.z = sin(heading_rad/2.0)
        quaternion.w = cos(heading_rad/2.0)
        self.odom.pose.pose.orientation = quaternion

        # publish to /odom
        self.pub.publish(self.odom)

        # publish to /tf
        if self.tf_frame != "":
            self.odomBroadcaster.sendTransform( (
                                        self.odom.pose.pose.position.x,
                                        self.odom.pose.pose.position.y,
                                        self.odom.pose.pose.position.z), 
                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    rospy.Time.now(),
                    "base_link",
                    self.tf_frame )

        # publish joint states
        self.joint_state.position = [left_wheel_angle_rad, right_wheel_angle_rad]
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.header.seq = sequence
        self.joint_pub.publish(self.joint_state)

        # publish odom_extra
        self.odom_extra.header.frame_id = 'odom'
        self.odom_extra.position.x = self.odom.pose.pose.position.x
        self.odom_extra.position.y = self.odom.pose.pose.position.y
        self.odom_extra.heading = heading_rad
        self.odom_extra.linear_speed = linear_speed
        self.odom_extra.angular_speed = self.odom.twist.twist.angular.z
        self.odom_extra.odometer = odometer
        self.odom_extra.left_speed = left_speed
        self.odom_extra.right_speed = right_speed
        self.odom_extra.left_encoder_cnt = left_enc_cnt
        self.odom_extra.right_encoder_cnt = right_enc_cnt
        self.odom_extra.odom_heading = odom_heading_rad
        self.odom_extra.IMUCalStatus = IMUCalStatus

        self.odom_extra.header.seq = sequence
        self.odom_extra.header.stamp = rospy.Time.now()
        self.odom_extra_pub.publish(self.odom_extra)

        # print odometry data to logs now and then
        self.log_cnt += 1
        if (self.log_cnt == self.log_rate):
            self.log_cnt = 0
            rospy.logdebug("Odometry poseX: {:.2f} poseY: {:.2f} compass heading: {:.2f} speed {:.2f}, odom: {:.2f}".format(
                self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, heading_rad, linear_speed, odometer))

        if sequence != (self.last_esp_seq + 1):
            rospy.logerr("RxOdometry detected {} lost msgs".format(sequence - self.last_esp_seq + 1))
        self.last_esp_seq = sequence

class RxPlatformData:
    def __init__(self, link):
        self.link = link

        # configure platform_data publishing
        self.platform_data_pub = rospy.Publisher('platform_data', PlatformData, queue_size=1)
        self.platform_data = PlatformData()
        self.last_platform_data_seq = 0

    def handle_platform_data(self):
        # rospy.logdebug('PlatformData callback got msg length: {}'.format(self.link.bytesRead))
        rec_size = 0

        # populate platform_ddata from PlatformDataMsg
        # member seq
        sequence = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        self.platform_data.header.seq = sequence
        if sequence != self.last_platform_data_seq:
            rospy.logerr('RxPlatformData detected lost msgs, seq: {}, expected {}'.format(sequence, self.last_platform_data_seq))
        self.last_platform_data_seq = sequence
        # member espTimestamp
        espTimestamp = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        # member leftMps
        self.platform_data.leftMps = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member rightMps
        self.platform_data.rightMps = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member leftPct
        leftPct = self.link.rx_obj(obj_type='i', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['i']
        self.platform_data.leftPct = leftPct
        # member rightPct
        rightPct = self.link.rx_obj(obj_type='i', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['i']
        self.platform_data.rightPct = rightPct
        # member commandedLinear
        self.platform_data.commandedLinear = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member commandedAngular
        self.platform_data.commandedAngular = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']

        self.platform_data.header.stamp = rospy.Time.now()
        if (abs(leftPct) > 100 or abs(leftPct) > 100):
            rospy.logerr('RxPlatformData magnitude error: leftPct {} rightPct {}'.format(self.platform_data.leftPct, self.platform_data.rightPct))
        self.platform_data_pub.publish(self.platform_data)
'''

class RxPong:
    def __init__(self, esp_link_node):
        self.esp_link_node = esp_link_node  # hold a reference to the esp_link node so we can use its node services and link
        self.link = self.esp_link_node.link

    def handle_pong(self):
        self.ping_type = b'\x00'
        self.timestamp = self.link.rx_obj(obj_type=type(self.timestamp), obj_byte_size=4, list_format='i')
        self.pong_type = self.link.rx_obj(obj_type=type(self.ping_type), obj_byte_size=1, list_format='b')
        self.esp_link_node.get_logger().info ("pong type {} timestamp: {}".format(self.pong_type, self.timestamp))
'''
class TxBITMode:
    def __init__(self, link):
        self.link = link
        self.posted = False

    def post(self):
        if self.posted:
            rospy.logerr('TxBITMode previously posted rqst still pending sending')
        rospy.loginfo('TxBITMode sending BIT Mode request to ESP32')
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        dummy_data = 0
        send_size = self.link.tx_obj(dummy_data)
        self.link.send(send_size, pktIdBITMode)
        self.posted = False

class TxClearOdom:
    def __init__(self, link):
        self.link = link
        self.posted = False

    def post(self):
        if self.posted:
            rospy.logerr('TxClearOdom previously posted rqst still pending sending')
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        dummy = 0           # must send something for SerialTransfer to work
        send_size = self.link.tx_obj(dummy)

        self.link.send(send_size, pktIdClearOdom)
        self.posted = False

class TxDriveMotorsRqst:
    def __init__(self, link):
        self.link = link
        self.posted = False
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lastRqstTime = time()
        self.seq = 0    # incrementing sequence # enables detecting dropped messages
        self.posted_seq = 0
        rospy.Subscriber("/cmd_vel", Twist, self.callback, queue_size=1)

    def callback(self, cmd_vel):
        # drop requests received less than 50ms since the last one
        if ((time() - self.lastRqstTime) < 0.05):
            # rospy.logwarn("Dropped too-soon motors rqst")
            return
        if self.posted:
            rospy.logwarn("TxDriveMotorsRqst request overrun, dropping request")
            return

        self.linear_vel = cmd_vel.linear.x
        self.angular_vel = cmd_vel.angular.z
        self.posted_seq = self.seq
        self.seq += 1
        self.lastRqstTime = time()
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        # send_size = self.link.tx_obj(self.left_drive_pct, val_type_override='b')
        send_size = self.link.tx_obj(self.posted_seq)
        send_size = self.link.tx_obj(self.linear_vel, start_pos=send_size)
        send_size = self.link.tx_obj(self.angular_vel, start_pos=send_size)
        self.link.send(send_size, pktIdDriveMotorsRqst)
        self.posted = False

class TxLogLevel:
    def __init__(self, link):
        self.link = link
        self.posted = False
        self.pilink_log_level = 4
        self.rl500_log_level = 4
        self.odom_log_level = 4

    def post(self, pilink_log_level, rl500_log_level, odom_log_level):
        if self.posted:
            rospy.logerr('TxLogLevel previously posted rqst still pending sending')
        self.pilink_log_level = pilink_log_level
        self.rl500_log_level = rl500_log_level
        self.odom_log_level = odom_log_level
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        send_size = self.link.tx_obj(self.pilink_log_level)
        send_size = self.link.tx_obj(self.rl500_log_level, start_pos=send_size)
        send_size = self.link.tx_obj(self.odom_log_level, start_pos=send_size)

        self.link.send(send_size, pktIdLogLevel)
        self.posted = False
'''

class TxPing:
    def __init__(self, esp_link_node):
        self.esp_link_node = esp_link_node  # hold a reference to the esp_link node so we can use its link object and node services
        self.link = self.esp_link_node.link
        self.posted = False

    def post(self):
        if self.posted:
            self.esp_link_node.get_logger().error('TxPing previously posted rqst still pending sending')
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        ping_type = '\x00'

        send_size = self.link.tx_obj(ping_type)
        self.link.send(send_size, pktIdPing)
        self.esp_link_node.get_logger().info('TxPing sent ping')
        self.posted = False

'''
class TxReboot:
    def __init__(self, link):
        self.link = link
        self.posted = False

    def post(self):
        if self.posted:
            rospy.logerr('TxReboot previously posted rqst still pending sending')
        rospy.loginfo('TxReboot sending reboot request to ESP32')
        self.posted = True
    
    def send_posted(self):
        if not self.posted:
            return
        dummy_data = 0
        send_size = self.link.tx_obj(dummy_data)
        self.link.send(send_size, pktIdReboot)
        self.posted = False
'''