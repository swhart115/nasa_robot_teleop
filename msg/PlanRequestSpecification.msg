# a low-dimensional ID to indicate which group
Int8 group_id

# a higher-dimensional ID to indicate which group
String group_name

# mask on which joints not to control in the plan
JointMask joint_mask

# the control frame to plan (usually end-effector or tool-frame, but not always)
String control_frame

# a list of waypoints for the controlled frame
PoseStamped[] waypoints

# duration of plan
float32 duration

# the number of waypoints to visualize in RViz (0 will show all, 1 will show just the last/goal point, any other number will evenly distribute between start and goal)
Int16 num_visualizaton_points

# tolerance mode
int8 angle_variance  #predefined sets of 6-dof tolerances, tool frames, and interpolation methods. Will probably add more later
int8 maximum_angle_variance  #set to 0 to copy angle_variance
  int8 COPY_DESIRED = 0   #ONLY USE WITH maximum_angle_variance!!
  int8 FULL_CYLINDER = 1   #CYLINDER types can rotate partially around the x axis of the tool frame, with minimal rotation on the other axes
  int8 HALF_CYLINDER = 2
  int8 QUARTER_CYLINDER = 3
  int8 EIGHTH_CYLINDER = 4
  int8 SIXTEENTH_CYLINDER = 5
  int8 SIXTEENTH_CONE = 6  #CONE types can rotate fully around the x axis of the tool frame, with the specified tolerance on the other axes
  int8 EIGHTH_CONE = 7
  int8 QUARTER_CONE = 8
  int8 HEMISPHERE = 9
  int8 SPHERE = 10
  int8 EXACT_ANGLE = 11 #Basic types have the same variance on all three axes of rotation. 
  int8 SIXTEENTH = 12
  int8 EIGHTH = 13
  int8 QUARTER = 14
  int8 HALF = 15
  int8 PRE_GRASP = 16 # variance in all three axes decreases as you get closer to the end
  int8 PRE_DRILL = 17 # variance in everything but roll decreases as you get closer to the end
  int8 DUAL_ARM = 18 # pi / 180, suggested for use as the dual arm desired tolerance
  int8 DUAL_ARM_MAX = 19 # pi / 45, suggested for use as the maximum dual arm tolerance

# interpolcation type
int8 interpolation_type
  int8 NORMAL = 1
  int8 OVERSAMPLE_2 = 2
  int8 OVERSAMPLE_4 = 4
  int8 OVERSAMPLE_8 = 8