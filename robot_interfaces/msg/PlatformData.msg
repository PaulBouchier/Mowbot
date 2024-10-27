# This message is used to publish platform data including motor parameters
std_msgs/Header header
float32 left_mps         # left wheel speed request (m/s)
float32 right_mps        # right wheel speed request (m/s)
int32 left_pct           # Percent power commanded to left wheel
int32 right_pct          # Percent power commanded to right wheel
float32 commanded_linear # last commanded linear speed
float32 commanded_angular # last commanded angular speed
int32 bumper_pressed      # a bumper is pressed
