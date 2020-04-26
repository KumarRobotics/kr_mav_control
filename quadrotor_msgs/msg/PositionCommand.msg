Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
float64 yaw
float64 yaw_dot
float64[3] kx
float64[3] kv

# use_msg_gains_flags: flags that can be set using the constants USE_MSG_GAINS_*
# to select which gains (kx, kv) from the message to use.  Note that by default
# the message gains are not used.
uint8 use_msg_gains_flags

uint8 USE_MSG_GAINS_NONE         = 0
uint8 USE_MSG_GAINS_POSITION_X   = 1
uint8 USE_MSG_GAINS_POSITION_Y   = 2
uint8 USE_MSG_GAINS_POSITION_Z   = 4
uint8 USE_MSG_GAINS_POSITION_ALL = 7 # sets all three of the above
uint8 USE_MSG_GAINS_VELOCITY_X   = 8
uint8 USE_MSG_GAINS_VELOCITY_Y   = 16
uint8 USE_MSG_GAINS_VELOCITY_Z   = 32
uint8 USE_MSG_GAINS_VELOCITY_ALL = 56 # sets all three of the above
