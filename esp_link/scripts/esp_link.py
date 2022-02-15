import time
import sys
import rospy

from pySerialTransfer import pySerialTransfer as txfer
from messages import TxPing, RxPong, RxOdometry, RxLog 

# import a module that will hold objects to be accessed from any module
sys.path.append('.')
import config

# instantiate message-hander objects
tx_ping = TxPing()
rx_pong = RxPong()
rx_odometry = RxOdometry()
rx_log = RxLog()

'''
list of callback functions to be called during tick. The index of the function
reference within this list must correspond to the packet ID. For instance, if
you want to call the function rx_pong() when you parse a packet with an ID of 0, you
would write the callback list with "rx_pong" being in the 0th place of the list:
'''
callback_list = [ rx_pong.handle_pong, rx_odometry.handle_odometry, rx_log.handle_log ]

def ping_callback(event):
    print('Sending ping')
    tx_ping.post()

if __name__ == '__main__':
    rospy.init_node('esp_link', disable_signals=True)
    esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyUSB0")
    #esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyAMA1")
    rospy.loginfo("esp_link using serial port: {}".format(esp_port_name))

    rospy.Timer(rospy.Duration(5.0), ping_callback)

    try:
        config.link = txfer.SerialTransfer(esp_port_name)
        config.link.set_callbacks(callback_list)
        config.link.open()
        time.sleep(2) # allow some time for the Arduino to completely reset
        rxMsgCnt = 0;
        
        while True:
            config.link.tick()

            if config.link.status < 0:
                if config.link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif config.link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif config.link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(config.link.status))

            tx_ping.send_posted()
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        try:
            config.link.close()
        except:
            pass
    
    except:
        import traceback
        traceback.print_exc()
        
        try:
            config.link.close()
        except:
            pass

