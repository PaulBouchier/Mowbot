# This message is used to publish various odometry-related data items that don't belong is Odometry
Header Header
geometry_msgs/Point position          # x, y, z in meters
float32 heading         # heading in radians, increases counterclockwise from x (East)
float32 linear_speed    # speed in the forward direction, m/s
float32 angular_speed   # rotational speed about z axis, rad/sec, positive is counterclockwise
float32 odometer        # distance traveled since esp32 reboot, meters
float32 left_speed      # speed of left wheel, m/s
float32 right_speed     # speed of right wheel, m/s
uint32 left_encoder_cnt # left encoder count in ticks
uint32 right_encoder_cnt # right encoder count in ticks