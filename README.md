This package can be used in conjunction with Robot's that have been configured to run with the MoveIt! toolbox.

Two robots that have this capability are the NASA Robonaut2 and Valkyrie robots.  To check out these sims,

R2:

[https://bitbucket.org/nasa_ros_pkg/nasa_r2_common](https://bitbucket.org/nasa_ros_pkg/nasa_r2_common) (hydro branch)
[https://bitbucket.org/nasa_ros_pkg/nasa_r2_simulator](https://bitbucket.org/nasa_ros_pkg/nasa_r2_simulator) (hydro branch)

Val:

[https://bitbucket.org/swhart115/nasa_v1_simulator](https://bitbucket.org/swhart115/nasa_v1_simulator)

To run the teleop nodes follow the following steps:

For R2, run the following three launch files in different terminals:

$ roslaunch r2_gazebo r2_gazebo.launch
$ roslaunch r2_moveit_config moveit_planning_execution.launch
$ roslaunch nasa_robot_teleop r2.launch

For R2 w/ Legs, run the following three launch files in different terminals:
$ roslaunch r2_gazebo r2_fullbody_gazebo.launch
$ roslaunch r2_fullbody_moveit_config moveit_planning_execution.launch
$ roslaunch nasa_robot_teleop r2_fullbody.launch

For Valkyrie, run the following three launch files in different terminals:
$ roslaunch v1_gazebo v1_gazebo.launch
$ roslaunch v1_moveit_config moveit_planning_execution.launch
$ roslaunch nasa_robot_teleop valkyrie.launch