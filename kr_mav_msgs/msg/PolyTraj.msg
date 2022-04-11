int16   mav_id
float32 mav_radius

# the trajectory id, starts from "1".
uint32 traj_id
uint8 order
time start_time

# the polynomial coecfficients of the trajectory.
float64 start_yaw
float64 final_yaw
float64[] bound_x
float64[] bound_y
float64[] bound_z
float32[] duration