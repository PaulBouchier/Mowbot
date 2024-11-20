import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_interfaces.msg import OdomExtra
from robot_interfaces.msg import FuseDebug

import math
import numpy as np

map_rate = 20  # rate to publish map updates

class FuseNavData(Node):


    def __init__(self):
        super().__init__('fuse_nav_data')

        self.fuse_mode = 1
        self.declare_parameter('fuse_mode_param', self.fuse_mode,
                               ParameterDescriptor(description='Fuse mode - 1: gps-only, 2: gps+IMU'))
        self.get_logger().info('fuse_nav_data started in mode {}'.format(self.fuse_mode))

        # subscribe to data sources
        self.utm_msg = PoseStamped()
        self.last_utm_msg = PoseStamped()
        self.utm_msg_count = 0
        self.create_subscription(PoseStamped, '/utm', self.utm_cb, 10)
        
        self.odom_extra_msg = OdomExtra()
        self.create_subscription(OdomExtra, '/odom_extra', self.odom_extra_cb, 10)

        self.odom_msg = Odometry()
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # publisher for fused result to /map
        self.map = PoseStamped()
        self.map.header.frame_id = 'map'
        self.gps_heading = 0.0
        self.q = self.quaternion_from_euler(0.0, 0.0, 0.0)
        self.map_pub = self.create_publisher(PoseStamped, 'map', 10)

        self.fuse_debug = FuseDebug()
        self.fuse_debug_pub = self.create_publisher(FuseDebug, 'fuse_debug', 10)

        self.create_timer(1.0/map_rate, self.update_map_cb)

    def utm_cb(self, msg):
        self.last_utm_msg = self.utm_msg
        self.utm_msg = msg
        self.utm_msg_count += 1
        self.fuse_debug.new_gps_data = True

        last_x = self.last_utm_msg.pose.position.x
        last_y = self.last_utm_msg.pose.position.y
        x = self.utm_msg.pose.position.x
        y = self.utm_msg.pose.position.y
        self.fuse_debug.gps_x = x
        self.fuse_debug.gps_y = y

        if (self.utm_msg_count > 1):  # must have received at least 2 measurements
            delta_x = x - last_x
            delta_y = y - last_y
            self.fuse_debug.gps_delta_x = delta_x
            self.fuse_debug.gps_delta_y = delta_y

            self.dist_moved = math.sqrt(delta_x**2 + delta_y**2)
            self.fuse_debug.dist_moved = self.dist_moved

            if (self.dist_moved > 0.05):    # must have moved at least 5cm to calculate new heading
                self.gps_heading = math.atan2(delta_y, delta_x)  # -pi to +pi from East
                self.q = self.quaternion_from_euler(0.0, 0.0, self.gps_heading)
                self.fuse_debug.gps_heading = self.gps_heading


    def odom_extra_cb(self, msg):
        self.odom_extra_msg = msg

    def odom_cb(self, msg):
        self.odom_msg = msg

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def update_map_cb(self):
        self.map.header.stamp = self.utm_msg.header.stamp
        self.map.pose.position.x = self.utm_msg.pose.position.x
        self.map.pose.position.y = self.utm_msg.pose.position.y
        self.map.pose.orientation.x = self.q[0]
        self.map.pose.orientation.y = self.q[1]
        self.map.pose.orientation.z = self.q[2]
        self.map.pose.orientation.w = self.q[3]

        self.map_pub.publish(self.map)

        self.fuse_debug_pub.publish(self.fuse_debug)
        self.fuse_debug.new_gps_data = False
        self.fuse_debug.new_odom_data = False
        self.fuse_debug.new_imu_data = False


def main(args=None):
    rclpy.init(args=args)

    fuse_nav_data = FuseNavData()

    try:
        rclpy.spin(fuse_nav_data)
    except KeyboardInterrupt:
        print("Shutting down after KeyboardInterrupt")

if __name__ == '__main__':
    main()