std_msgs/Header header
int16   total_agent_num
int16   agent_id
float32 agent_radius
# the trajectory id, starts from "1".
uint32 traj_id
uint8 order # also has degree infos
uint8 dim
time start_time

# the polynomial coecfficients of the trajectory.
float64[] coeff_x
float64[] coeff_y
float64[] coeff_z
float32[] duration

# seg * (order + 1) * dim,   with the  [a0, a1, a2, a3, a4, a5] , 
# a0 is the constant term, a1 is the linear term, a2 is the quadratic term, and so on.