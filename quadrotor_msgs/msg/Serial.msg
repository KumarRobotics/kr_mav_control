# Note: These constants need to be kept in sync with the types
# defined in include/quadrotor_msgs/comm_types.h
uint8 SO3_CMD = 115 # 's' in base 10
uint8 TRPY_CMD = 112 # 'p' in base 10
uint8 PWM_CMD = 119 # 'w' in base 10
uint8 STATUS_DATA = 99 # 'c' in base 10
uint8 OUTPUT_DATA = 100 # 'd' in base 10

Header header
uint8 channel
uint8 type # One of the types listed above
uint8[] data
