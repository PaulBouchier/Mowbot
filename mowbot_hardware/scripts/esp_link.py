#!/usr/bin/env python
import time
import sys
import rospy
from pySerialTransfer import pySerialTransfer as txfer
from messages import RxLog, RxOdometry, RxPlatformData, RxPong, \
     TxBITMode, TxClearOdom, TxDriveMotorsRqst, TxLogLevel, TxPing, TxReboot
from dynamic_reconfigure.server import Server
from mowbot_hardware.cfg import MowbotConfig

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

# initialize link
def init_link(port_name):
    try:
        global link
        link = txfer.SerialTransfer(esp_port_name)
    except:
        import traceback
        traceback.print_exc()
        print("Exception in init_link, exiting")
        sys.exit()

# initialize messages
def init_msgs(link):
    global rx_log, rx_odometry, rx_platform_data, rx_pong, \
        tx_bit_mode, tx_clear_odom, tx_drive_motors_rqst, tx_log_level, tx_ping, tx_reboot 
    # instantiate rx & tx message handlers
    rx_log = RxLog(link)
    rx_odometry = RxOdometry(link)
    rx_platform_data = RxPlatformData(link)
    rx_pong = RxPong(link)

    tx_bit_mode = TxBITMode(link)
    tx_clear_odom = TxClearOdom(link)
    tx_drive_motors_rqst = TxDriveMotorsRqst(link)
    tx_log_level = TxLogLevel(link)
    tx_ping = TxPing(link)
    tx_reboot = TxReboot(link)

def tick_link():
    start_time = time.time()
    link.tick()
    end_time = time.time()
    #print('tick_link: {} ms'.format((end_time - start_time)*1000))
    if link.status < 0:
        if link.status == txfer.CRC_ERROR:
            print('ERROR: CRC_ERROR')
        elif link.status == txfer.PAYLOAD_ERROR:
            print('ERROR: PAYLOAD_ERROR')
        elif link.status == txfer.STOP_BYTE_ERROR:
            print('ERROR: STOP_BYTE_ERROR')
        else:
            print('ERROR: {}'.format(link.status))

def ping_callback(event):
    tx_ping.post()

def reconfig_callback(config, level):
    global pilink_log_lvl, rl500_log_lvl, odom_log_lvl

    rospy.logdebug('Reconfig log levels: pilink: {pilink_log_lvl}, rl500: {rl500_log_lvl}, odom: {odom_log_lvl}'.format(**config))
    rospy.logdebug('Reconfig PID: use_pid: {use_pid}, prop gain: {pid_p}, integral: {pid_i}, derivative: {pid_d}'.format(**config))
    rospy.logdebug('Reconfig ESP32: bit_mode: {rl500_bit_mode}, reboot: {esp_reboot}, clear_odometry: {clear_odometry}'.format(**config))

    if config['pilink_log_lvl'] != pilink_log_lvl or config['rl500_log_lvl'] != rl500_log_lvl or config['odom_log_lvl'] != odom_log_lvl:
        pilink_log_lvl = config['pilink_log_lvl']
        rl500_log_lvl = config['rl500_log_lvl']
        odom_log_lvl = config['odom_log_lvl']
        tx_log_level.post(pilink_log_lvl, rl500_log_lvl, odom_log_lvl)

    if config['esp_reboot']:
        tx_reboot.post()
        config['esp_reboot'] = False

    if config['rl500_bit_mode']:
        tx_bit_mode.post()
        config['rl500_bit_mode'] = False

    if config['clear_odometry']:
        tx_clear_odom.post()
        config['clear_odometry'] = False

    return config

if __name__ == '__main__':
    rospy.init_node('esp_link', disable_signals=True, log_level=rospy.DEBUG)
    #esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyUSB0")
    esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyAMA1")
    rospy.loginfo("esp_link using serial port: {}".format(esp_port_name))

    rospy.Timer(rospy.Duration(5.0), ping_callback)

    try:
        # initialize pySerialTransfer
        init_link(esp_port_name)
        init_msgs(link)
        # configure dynamic reconfigure
        srv = Server(MowbotConfig, reconfig_callback)

        
        '''
        list of callback functions to be called during tick. The index of the function
        reference within this list must correspond to the packet ID. For instance, if
        you want to call the function rx_pong() when you parse a packet with an ID of 0, you
        would write the callback list with "rx_pong" being in the 0th place of the list:
        '''
        callback_list = [ rx_pong.handle_pong, \
                        rx_odometry.handle_odometry, \
                        rx_log.handle_log, \
                        rx_platform_data.handle_platform_data ]

        link.set_callbacks(callback_list)
        link.open()

        time.sleep(2) # allow some time for the Arduino to completely reset
        
        while True:
            tick_link()
            tx_bit_mode.send_posted()
            tick_link()
            tx_clear_odom.send_posted()
            tick_link()
            tx_drive_motors_rqst.send_posted()
            tick_link()
            tx_log_level.send_posted()
            tick_link()
            tx_ping.send_posted()
            tick_link()
            tx_reboot.send_posted()
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        try:
            link.close()
        except:
            pass
    
    except:
        import traceback
        traceback.print_exc()
        
        try:
            link.close()
        except:
            pass

