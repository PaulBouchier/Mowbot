import sys
import rospy
from time import sleep, time
from math import sin, cos
from pySerialTransfer import pySerialTransfer as txfer

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster

MAX_SPEED = 0.47      # meters/second
BASE_WIDTH = 0.424    # meters, 16.675"
WHEEL_RADIUS = 0.127  # meters

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

        self.ros_odom_seq = 0
        self.last_esp_seq = 0

        self.log_rate = 1000    # print a log message every log_rate OdometryMsg's
        self.log_cnt = 0

    def handle_odometry(self):
        # rospy.logdebug ('Odometry callback got msg length: {}'.format(self.link.bytesRead))
        self.odom.header.seq = self.ros_odom_seq
        self.odom.header.stamp = rospy.Time.now()
        rec_size = 0

        # populate odom from OdometryMsg
        # member seq
        sequence = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        # member poseX_m
        self.odom.pose.pose.position.x = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member poseY_m
        self.odom.pose.pose.position.y = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        # member heading_rad
        heading_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)
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
        left_enc_cnt = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        # member rightEncoderCount
        right_enc_cnt = self.link.rx_obj(obj_type='I', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['I']
        #member leftWheelAngleRad
        left_wheel_angle_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']
        #member rightWheelAngleRad
        right_wheel_angle_rad = self.link.rx_obj(obj_type='f', start_pos=rec_size)
        rec_size += txfer.STRUCT_FORMAT_LENGTHS['f']

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
                    rospy.Time(),
                    "base_link",
                    self.tf_frame )

        # publish joint states
        self.joint_state.position = [left_wheel_angle_rad, right_wheel_angle_rad]
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.header.seq = self.ros_odom_seq
        self.joint_pub.publish(self.joint_state)
        self.ros_odom_seq += 1

        # print odometry data to logs now and then
        self.log_cnt += 1
        if (self.log_cnt == self.log_rate):
            self.log_cnt = 0
            rospy.loginfo("poseX: {:.2f} poseY: {:.2f} heading: {:.2f} speed {:.2f}, odom: {:.2f}".format(
                self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, heading_rad, linear_speed, odometer))

        if sequence != (self.last_esp_seq + 1):
            rospy.logerr("Detected {} dropped odom messages".format(sequence - self.last_esp_seq + 1))
        self.last_esp_seq = sequence

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
        self.link.send(send_size, 1)
        self.posted = False

