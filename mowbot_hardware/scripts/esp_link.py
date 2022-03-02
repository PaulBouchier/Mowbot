#!/usr/bin/env python
import time
import sys
import rospy
from pySerialTransfer import pySerialTransfer as txfer
from messages import TxPing, TxDriveMotorsRqst, RxPong, RxOdometry, RxLog, TxLogLevel, TxReboot
from dynamic_reconfigure.server import Server
from mowbot_hardware.cfg import MowbotConfig

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
    global tx_ping, rx_pong, rx_odometry, tx_drive_motors_rqst, tx_log_level, tx_reboot
    tx_ping = TxPing(link)
    rx_pong = RxPong(link)
    rx_odometry = RxOdometry(link)
    tx_drive_motors_rqst = TxDriveMotorsRqst(link)
    rx_log = RxLog(link)
    tx_log_level = TxLogLevel(link)
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
    print('Sending ping')
    tx_ping.post()

def drive_motors_callback(event):
    # OBSOLETE - unused
    speed = [50, 50]
    tx_drive_motors_rqst.post(speed[0], speed[1])
    print('Sending motors')

pilink_log_lvl = 4
rl500_log_lvl = 4
odom_log_lvl = 4
use_pid = False
pid_p = 1.0
pid_i = 1.0
pid_d = 1.0
esp_reboot = False

def reconfig_callback(config, level):
    rospy.loginfo('Reconfig log levels: pilink: {pilink_log_lvl}, rl500: {rl500_log_lvl}, odom: {odom_log_lvl}'.format(**config))
    rospy.loginfo('Reconfig PID: use_pid: {use_pid}, prop gain: {pid_p}, integral: {pid_i}, derivative: {pid_d}'.format(**config))
    rospy.loginfo('Reconfig ESP32: reboot: {esp_reboot}'.format(**config))
    return config

if __name__ == '__main__':
    rospy.init_node('esp_link', disable_signals=True)
    #esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyUSB0")
    esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyAMA1")
    rospy.loginfo("esp_link using serial port: {}".format(esp_port_name))

    # configure dynamic reconfigure
    srv = Server(MowbotConfig, reconfig_callback)

    rospy.Timer(rospy.Duration(5.0), ping_callback)
    #rospy.Timer(rospy.Duration(0.5), drive_motors_callback)

    try:
        # initialize pySerialTransfer
        init_link(esp_port_name)
        init_msgs(link)
        
        '''
        list of callback functions to be called during tick. The index of the function
        reference within this list must correspond to the packet ID. For instance, if
        you want to call the function rx_pong() when you parse a packet with an ID of 0, you
        would write the callback list with "rx_pong" being in the 0th place of the list:
        '''
        callback_list = [ rx_pong.handle_pong, rx_odometry.handle_odometry, rx_log.handle_log ]

        link.set_callbacks(callback_list)
        link.open()

        time.sleep(2) # allow some time for the Arduino to completely reset
        
        while True:
            tick_link()
            tx_ping.send_posted()
            tick_link()
            tx_drive_motors_rqst.send_posted()
            tick_link()
            tx_log_level.send_posted()
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

