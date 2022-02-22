import time
import sys
import rospy
from pySerialTransfer import pySerialTransfer as txfer
from messages import TxPing, TxDriveMotorsRqst, RxPong, RxOdometry, RxLog

# import a module that will hold objects to be accessed from any module
sys.path.append('.')
#import config

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
    global tx_ping, rx_pong, rx_odometry, tx_drive_motors_rqst, rx_log
    tx_ping = TxPing(link)
    #global rx_pong
    rx_pong = RxPong(link)
    rx_odometry = RxOdometry(link)
    tx_drive_motors_rqst = TxDriveMotorsRqst(link)
    rx_log = RxLog(link)


def ping_callback(event):
    print('Sending ping')
    tx_ping.post()

def drive_motors_callback(event):
    speed = [50, 50]
    tx_drive_motors_rqst.post(speed[0], speed[1])
    print('Sending motors')

if __name__ == '__main__':
    rospy.init_node('esp_link', disable_signals=True)
    #esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyUSB0")
    esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyAMA1")
    rospy.loginfo("esp_link using serial port: {}".format(esp_port_name))

    rospy.Timer(rospy.Duration(5.0), ping_callback)
    rospy.Timer(rospy.Duration(0.5), drive_motors_callback)

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
            link.tick()

            if link.status < 0:
                if link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status))

            tx_ping.send_posted()
            tx_drive_motors_rqst.send_posted()
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

