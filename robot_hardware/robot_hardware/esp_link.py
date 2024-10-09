#!/usr/bin/env python
import time
import sys
import rclpy
from rclpy.node import Node
from pySerialTransfer import pySerialTransfer as txfer

# FIXME
from robot_hardware.messages import RxPong, TxPing
'''
from messages import RxLog, RxOdometry, RxPlatformData, RxPong, \
     TxBITMode, TxClearOdom, TxDriveMotorsRqst, TxLogLevel, TxPing, TxReboot
'''

pilink_log_lvl = 4
rl500_log_lvl = 4
odom_log_lvl = 4
use_pid = False
pid_p = 1.0
pid_i = 1.0
pid_d = 1.0
rl500_bit_mode = False
esp_reboot = False
clear_odom = False

class EspLink(Node):

    def __init__(self):
        super().__init__('esp_link')

        #esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyUSB0")
        esp_port_name = "/dev/ttyUSB0"
        self.get_logger().info("esp_link using serial port: {}".format(esp_port_name))

        try:
            # initialize pySerialTransfer
            self.init_link(esp_port_name)
            self.init_msgs()

            
            '''
            list of callback functions to be called during tick. The index of the function
            reference within this list must correspond to the packet ID. For instance, if
            you want to call the function rx_pong() when you parse a packet with an ID of 0, you
            would write the callback list with "rx_pong" being in the 0th place of the list:
            '''
            '''
            callback_list = [ rx_pong.handle_pong, \
                            rx_odometry.handle_odometry, \
                            rx_log.handle_log, \
                            rx_platform_data.handle_platform_data ]
            '''
            callback_list = [ self.rx_pong.handle_pong, \
                            ]

            self.link.set_callbacks(callback_list)
            self.link.open()
        except:
            import traceback
            traceback.print_exc()
            
            try:
                link.close()
            except:
                pass


        time.sleep(2) # allow some time for the Esp32 to completely reset
            
        self.ping_timer = self.create_timer(5, self.ping_timer_callback)  # ping esp32 every 5 sec
        self.link_tick_timer = self.create_timer(0.01, self.link_tick_timer_callback)  # tick the link at 100 Hz

    # initialize link
    def init_link(self, esp_port_name):
        try:
            self.link = txfer.SerialTransfer(esp_port_name)
            self.get_logger().info("Initialized link")
        except Exception as exception:
            import traceback
            traceback.print_exc()
            self.get_logger().fatal("Exception in init_link, exiting")
            raise exception

    # initialize messages
    def init_msgs(self):
        '''
        global rx_log, rx_odometry, rx_platform_data, rx_pong, \
            tx_bit_mode, tx_clear_odom, tx_drive_motors_rqst, tx_log_level, tx_ping, tx_reboot 
        '''
        # instantiate rx & tx message handlers
        '''
        rx_log = RxLog(link)
        rx_odometry = RxOdometry(link)
        rx_platform_data = RxPlatformData(link)
        '''
        self.rx_pong = RxPong(self)  # pass the esp_link object to RxPong so it can access link, create Node objects, etc

        '''
        tx_bit_mode = TxBITMode(link)
        tx_clear_odom = TxClearOdom(link)
        tx_drive_motors_rqst = TxDriveMotorsRqst(link)
        tx_log_level = TxLogLevel(link)
        tx_reboot = TxReboot(link)
        '''
        self.tx_ping = TxPing(self)

    def tick_link(self):
        start_time = time.time()
        self.link.tick()
        end_time = time.time()
        # self.get_logger().info('tick_link: {} ms'.format((end_time - start_time)*1000))
        if self.link.status < 0:
            if self.link.status == txfer.Status.CRC_ERROR:
                self.get_logger().warn('ERROR: CRC_ERROR')
            elif self.link.status == txfer.Status.PAYLOAD_ERROR:
                self.get_logger().warn('ERROR: PAYLOAD_ERROR')
            elif self.link.status == txfer.Status.STOP_BYTE_ERROR:
                self.get_logger().warn('ERROR: STOP_BYTE_ERROR')
            else:
                self.get_logger().warn('ERROR: Unknown degative link status: {}'.format(link.status.value))

    def link_tick_timer_callback(self):
        '''
        tx_bit_mode.send_posted()
        self.tick_link()
        tx_clear_odom.send_posted()
        self.tick_link()
        tx_drive_motors_rqst.send_posted()
        self.tick_link()
        tx_log_level.send_posted()
        self.tick_link()
        tx_reboot.send_posted()
        self.tick_link()
        '''
        self.tx_ping.send_posted()
        self.tick_link()
        
    def ping_timer_callback(self):
        self.get_logger().info("posting ping")
        self.tx_ping.post()

def main(args=None):
    rclpy.init(args=args)

    esp_link = EspLink()

    rclpy.spin(esp_link)

    esp_link.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()