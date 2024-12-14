import rclpy
from rclpy.node import Node

from rclpy import qos
from ublox_ubx_msgs.msg import UBXNavHPPosLLH, UBXNavStatus
from geometry_msgs.msg import PoseStamped
from robot_interfaces.msg import OdomExtra

import math


class UbloxDgnssSubscriber(Node):

    def __init__(self):
        super().__init__('ublox_dgnss_subscriber')

        self.dist_moved = 0.0
        self.gps_heading = 0.0

        self.last_print_time = self.get_clock().now().nanoseconds
        self.last_hp_pos_llh_msg_time = int(self.get_clock().now().nanoseconds/1000000000)

        self.ubx_nav_hp_pos_llh = UBXNavHPPosLLH()
        self.create_subscription(
            UBXNavHPPosLLH,
            'ubx_nav_hp_pos_llh',
            self.ubx_nav_hp_pos_llh_cb,
            qos.qos_profile_sensor_data)

        self.ubx_nav_status = UBXNavStatus()
        self.create_subscription(
            UBXNavStatus,
            'ubx_nav_status',
            self.ubx_nav_status_cb,
            qos.qos_profile_sensor_data)

        self.utm_msg = PoseStamped()
        self.utm_last_msg = PoseStamped()
        self.create_subscription(
            PoseStamped,
            '/utm',
            self.utm_cb,
            10
        )
        
        self.odom_extra_msg = OdomExtra
        self.create_subscription(
            OdomExtra,
            '/odom_extra',
            self.odom_extra_cb,
            10
        )
        
        self.create_timer(1.0, self.print_cb)

    def ubx_nav_hp_pos_llh_cb(self, msg):
        # self.get_logger().info('h_acc: "%d"' % msg.h_acc)
        self.ubx_nav_hp_pos_llh = msg
        self.last_hp_pos_llh_msg_time = int(self.get_clock().now().nanoseconds/1000000000)

    def ubx_nav_status_cb(self, msg):
        # self.get_logger().info('h_acc: "%d"' % msg.h_acc)
        self.ubx_nav_status = msg
        self.last_status_msg_time = int(self.get_clock().now().nanoseconds/1000000000)

    def utm_cb(self, msg):
        self.utm_last_msg = self.utm_msg
        self.utm_msg = msg
        last_x = self.utm_last_msg.pose.position.x
        last_y = self.utm_last_msg.pose.position.y
        x = self.utm_msg.pose.position.x
        y = self.utm_msg.pose.position.y
        if ((last_x != 0 or last_y != 0)  # must have received at least 2 measurements
            and (x != last_x or y != last_y)):  # must have moved
            delta_x = x - last_x
            delta_y = y - last_y

            self.dist_moved = math.sqrt(delta_x**2 + delta_y**2)
            self.gps_heading = math.atan2(delta_y, delta_x) + math.pi  # 0-two pi from East

    def odom_extra_cb(self, msg):
        self.odom_extra_msg = msg

    def print_cb(self):
        self.now = int(self.get_clock().now().nanoseconds/1000000000)
        if (self.now - self.last_hp_pos_llh_msg_time < 10):
            print('E: {0:.2f} N: {1:.2f}'.format(self.utm_msg.pose.position.x, self.utm_msg.pose.position.y), end=' ')
            # The following print is only supported when the esp_link node is publishing odom_extra
            # print('hdg: {0:.2f}'.format(self.odom_extra_msg.heading), end=' ')
            print('gps_hdg: {0:.2f}'.format(self.gps_heading), end=' ')
            print('delta_xy: {0:.2f}'.format(self.dist_moved), end=' ')
            print('h_acc: {0:.1f} cm'.format(self.ubx_nav_hp_pos_llh.h_acc/100), end=' ')
            print('gps_fix_ok: {} diff_soln: {} diff_corr: {}'.format(
                self.ubx_nav_status.gps_fix_ok,
                self.ubx_nav_status.diff_soln,
                self.ubx_nav_status.diff_corr),
                end=' ')
        else:
            print('\nSTALE')
        # print('at time: {}'.format(int(self.last_print_time/1000000000)), end=' ')
        print('', end='\r')
        self.last_print_time = self.get_clock().now().nanoseconds
        


def main(args=None):
    rclpy.init(args=args)

    ublox_dgnss_subscriber = UbloxDgnssSubscriber()

    rclpy.spin(ublox_dgnss_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ublox_dgnss_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
