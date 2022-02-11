import time
import sys
import serial
import rospy
from pySerialTransfer import pySerialTransfer as txfer


class RxPong:
    def handle_pong(self):
        print ('Pong callback')
        self.ping_type = b'\x00'
        self.timestamp = 0
        self.pong_type = link.rx_obj(obj_type=type(sefl.ping_type), obj_byte_size=1, list_format='b')
        self.timestamp = link.rx_obj(obj_type=type(self.timestamp), obj_byte_size=4, list_format='i')
        print ("pong type {} timestamp: {}".format(self.pong_type, self.timestamp))

rx_pong = RxPong()

def pong_cb():
    rx_pong.handle_pong()

class TxPing:
    def __init__(self):
        self.posted = False

    def post(self):
        self.posted = True
    
    def send_posted(self):
        ping_type = b'\x00'
        send_size = link.tx_obj(ping_type)
        link.send(send_size, 0)
        
tx_ping = TxPing()

'''
list of callback functions to be called during tick. The index of the function
reference within this list must correspond to the packet ID. For instance, if
you want to call the function rx_pong() when you parse a packet with an ID of 0, you
would write the callback list with "rx_pong" being in the 0th place of the list:
'''
callback_list = [ rx_pong ]

if __name__ == '__main__':
    rospy.init_node('esp_link')
    esp_port_name = rospy.get_param('~esp_port_name', "/dev/ttyAMA1")
    rospy.loginfo("esp_link using serial port: {}".format(esp_port_name))

    try:
        global link
        link = txfer.SerialTransfer('/dev/ttyAMA1')
        link.set_callbacks(callback_list)
        link.open()
        time.sleep(2) # allow some time for the Arduino to completely reset
        rxMsgCnt = 0;
        
        while True:

            if link.status < 0:
                if link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status))

            time.sleep(0.5)
    
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

