# a low-dimensional ID to indicate which group
int8 group_id

# a higher-dimensional ID to indicate which group
string group_name

# mask on which joints not to control in the plan
JointMask joint_mask

# the control frame to plan (usually end-effector or tool-frame, but not always)
string control_frame

# a list of waypoints for the controlled frame
geometry_msgs/PoseStamped[] waypoints

# duration of plan (per waypoint)
float32[] duration

# the number of waypoints to visualize in RViz (0 will show all, 1 will show just the last/goal point, any other number will evenly distribute between start and goal)
int16 num_visualizaton_points

# angle tolerance mode
geometry_msgs/Vector3[] angle_variance  #predefined sets of 6-dof tolerances, tool frames, and interpolation methods. Will probably add more later
geometry_msgs/Vector3[] maximum_angle_variance  #set to "" to copy angle_variance

# position tolerance mode
geometry_msgs/Vector3[] position_variance  #predefined sets of 6-dof tolerances, tool frames, and interpolation methods. Will probably add more later
geometry_msgs/Vector3[] maximum_position_variance  #set to "" to copy angle_variance

# interpolcation type
int8 interpolation_type
  int8 NORMAL = 1
  int8 OVERSAMPLE_2 = 2
  int8 OVERSAMPLE_4 = 4
  int8 OVERSAMPLE_8 = 8