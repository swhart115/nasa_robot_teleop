#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import moveit_commander

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg
import actionlib_msgs.msg

from nasa_robot_teleop.path_planner import *

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *

class MoveItPathPlanner(PathPlanner) :

    ############################
    ####### CONSTRUCTOR ########   
    ############################

    def __init__(self, robot_name, config_file):
        PathPlanner.__init__(self, robot_name, config_file)

        self.config_file = config_file

        s = self.config_file[:self.config_file.rfind("/")]
        self.config_package = s[:s.rfind("/")]

        # connect to moveit server
        rospy.loginfo(str("============ Setting up MoveIt! for robot: \'" + self.robot_name + "\'"))
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.groups = {}
        
        self.actionlib = False

        # set up obstacle publisher stuff
        self.obstacle_markers = visualization_msgs.msg.MarkerArray()
        self.obstacle_publisher = rospy.Publisher(str('/' + self.robot_name + '/obstacle_markers'), visualization_msgs.msg.MarkerArray, queue_size=10)
        
        rospy.loginfo(str("============ Setting up MoveIt! for robot: \'" + self.robot_name + "\' finished"))
        
    ##############################
    ####### SETUP METHODS ########   
    ##############################

    def setup_group(self, group_name, joint_tolerance, position_tolerances, orientation_tolerances) :
        r = True
        rospy.loginfo(str("MoveItPathPlanner::setup_group() -- " + group_name))     

        self.position_tolerances[group_name] = position_tolerances
        self.orientation_tolerances[group_name] = orientation_tolerances
        self.joint_tolerance[group_name] = joint_tolerance

        try :
            controller_name = self.lookup_controller_name(group_name)  # FIXME
            msg_type = control_msgs.msg.FollowJointTrajectoryActionGoal
            bridge_topic_name = self.lookup_bridge_topic_name(controller_name)
            if bridge_topic_name != "" :
                from pr2_joint_trajectory_bridge.msg import JointTrajectoryBridge
                controller_name = bridge_topic_name
                msg_type = JointTrajectoryBridge
                self.bridge_topic_map[group_name] = bridge_topic_name

            # topic_name = "/" + controller_name + "/command"
            topic_name = "/" + controller_name + "/goal"
            rospy.loginfo(str("COMMAND TOPIC: " + topic_name))
            self.command_topics[group_name] = rospy.Publisher(topic_name, msg_type, queue_size=10)
        except :
            rospy.logerr(str("MoveItInterface()::setup_group() -- Robot " + self.robot_name + " has problem setting up controller for: " + group_name))
            r = False

        try :
            self.groups[group_name] = moveit_commander.MoveGroupCommander(group_name)
            self.groups[group_name].set_goal_joint_tolerance(joint_tolerance)

            if len(self.position_tolerances[group_name]) != 3 or len(self.orientation_tolerances[group_name]) != 3 :
                rospy.logwarn("MoveItPathPlanner::setup_group() tolerance vectors of wrong size. Just using first val")
                self.position_tolerances[group_name] = [self.position_tolerances[group_name][0]]*3
                self.orientation_tolerances[group_name] = [self.orientation_tolerances[group_name][0]]*3
            else :
                if not(self.position_tolerances[group_name][0] == self.position_tolerances[group_name][1] == self.position_tolerances[group_name][2]) :
                    self.position_tolerances[group_name] = [self.position_tolerances[group_name][0]]*3
                    rospy.logwarn("MoveItPathPlanner::setup_group() dimensional position tolerances not supported. Just using first val")
                if not(self.orientation_tolerances[group_name][0] == self.orientation_tolerances[group_name][1] == self.orientation_tolerances[group_name][2]) :
                    self.orientation_tolerances[group_name] = [self.orientation_tolerances[group_name][0]]*3
                    rospy.logwarn("MoveItPathPlanner::setup_group() dimensional orientation tolerances not supported. Just using first val")                              
            self.groups[group_name].set_goal_position_tolerance(position_tolerances[0])
            self.groups[group_name].set_goal_orientation_tolerance(self.orientation_tolerances[group_name][0])
        except :
            rospy.logerr(str("MoveItInterface()::setup_group() -- Robot " + self.robot_name + " has problem setting up MoveIt! commander group for: " + group_name))
            r = False
        return r 

     # this is where parse the PathPlanner config package for the robot and figure out what the controller names are 
    def lookup_controller_name(self, group_name) :

        if not group_name in self.group_controllers.keys() :
            import yaml
            try:
                controllers_file = self.config_package + "/config/controllers.yaml"
                rospy.logdebug("Controller yaml: " + controllers_file)
                controller_config = yaml.load(file(controllers_file, 'r'))
            except :
                rospy.logerr("PathPlanner::lookup_controller_name() -- Error loading controllers.yaml")
            joint_list = self.get_group_joints(group_name)
            jn = joint_list[0]
            for j in joint_list:
                if j in self.urdf_model.joint_map :
                    if self.urdf_model.joint_map[j].type != "fixed" :
                        jn = j
                        break
            self.group_controllers[group_name] = ""
            for c in controller_config['controller_list'] :
                if jn in c['joints'] :
                    self.group_controllers[group_name] = c['name'] + "/" + c['action_ns']

        rospy.logdebug(str("PathPlanner::lookup_controller_name() -- Found Controller " + self.group_controllers[group_name]  + " for group " + group_name))
        return self.group_controllers[group_name]

    #################################
    ####### OBSTACLE METHODS ########   
    #################################

    def add_obstacle(self, p, s, n) :
        p.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box(n, p, s)
        m = visualization_msgs.msg.Marker()
        m.header.frame_id = p.header.frame_id
        m.type = m.CUBE
        m.action = m.ADD
        m.scale.x = s[0]
        m.scale.y = s[1]
        m.scale.z = s[2]
        m.color.a = 0.8
        m.color.r = 0
        m.color.g = 1
        m.color.b = 0
        m.pose = p.pose
        m.text = n
        m.ns = n
        self.obstacle_markers.markers.append(m)
        if self.obstacle_publisher :
            self.obstacle_publisher.publish(self.obstacle_markers)
        else :
            rospy.logerr("MoveItPathPlanner::add_obstacle() -- failed publishing obstacles")
       

    ################################
    ######## HELPER METHODS ########
    ################################
    
    def get_robot_planning_frame(self) :
        if self.robot :
            return self.robot.get_planning_frame()
        else :
            rospy.logerr(str("MoveItPathPlanner::get_robot_planning_frame() -- no robot set yet"))
            return None

    def get_group_planning_frame(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::get_group_planning_frame() -- group name \'" + str(group_name) + "\' not found"))
            return ""
        else :
            return self.groups[group_name].get_planning_frame()

    def has_end_effector_link(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::has_end_effector_link() -- group name \'" + str(group_name) + "\' not found"))
            return False
        else :
            return self.groups[group_name].has_end_effector_link()

    def get_end_effector_link(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::get_end_effector_link() -- group name \'" + str(group_name) + "\' not found"))
            return ""
        else :
            return self.groups[group_name].get_end_effector_link()

    def clear_goal_target(self, group_name) :
        try :
            self.groups[group_name].clear_pose_targets()
        except :
            rospy.logwarn(str("MoveItPathPlanner::clear_goal_target(" + group_name + ") -- failed"))

    def get_group_joints(self, group_name) :
        if self.robot :
            return self.robot.get_joint_names(group_name)
        else :
            rospy.logerr(str("MoveItPathPlanner::get_group_joints() -- group name \'" + str(group_name) + "\' not found"))
            return [] 

    def get_joint_map(self, group_name) :
       
        root = self.srdf_model.get_base_link(group_name)
        tip = self.srdf_model.get_tip_link(group_name)
        joint_list = []

        joint_list = self.srdf_model.get_group_joints(group_name)

        joint_name_map = nasa_robot_teleop.msg.JointNameMap()

        idx = 0
        for j in joint_list :
            joint_name_map.names.append(j)
            joint_name_map.ids.append(idx)
            idx += 1

        return joint_name_map


    ###################################
    ######## EXECUTION METHODS ########
    ###################################
  
    def execute_plans(self, group_names, from_stored=False, wait=True) :
        for group_name in group_names :
            if self.plan_generated[group_name] and self.stored_plans[group_name] :
                rospy.loginfo(str("MoveItPathPlanner::execute_plan() -- executing plan for group: " + group_name))
                if self.actionlib :
                    rospy.logdebug("MoveItPathPlanner::execute_plan() -- using actionlib")
                    r = self.go(group_name, wait)
                else :
                    rospy.logdebug("MoveItPathPlanner::execute_plan() -- publishing to topic")
                    jt = self.translate_trajectory_msg(group_name, self.stored_plans[group_name])             
                    N = len(jt.goal.trajectory.points)
                    rospy.logwarn(str("executing path of " + str(N) + " points"))
                    self.command_topics[group_name].publish(jt)
                    self.plan_generated[group_name] = False
                    r = True # no better way for monitoring success here as it is just an open-loop way of publishing the path
            else :
                rospy.logwarn(str("MoveItPathPlanner::execute_plan() -- no plan for group " + group_name + " yet generated."))
                r = False
            rospy.logdebug(str("MoveItPathPlanner::execute_plan() -- plan execution: " + str(r)))

    # This is the main execution function for sending trajectories to the robot.
    # This will only return True if the plan has been previously (successfully) generated.
    # It will allow the goal to be published using actionlib (which doesnt work for commanding multiple groups at once cause the pything moveit api blocks)
    # or directly to a goal topic (which makes it kind of open loop).
    def execute_plan(self, group_name, from_stored=False, wait=True) :
        r = False
        if self.plan_generated[group_name] and self.stored_plans[group_name] :
            rospy.loginfo(str("MoveItPathPlanner::execute_plan() -- executing plan for group: " + group_name))
            if self.actionlib :
                rospy.logdebug("MoveItPathPlanner::execute_plan() -- using actionlib")
                r = self.go(group_name, wait)
            else :
                rospy.logdebug("MoveItPathPlanner::execute_plan() -- publishing to topic")
                jt = self.translate_trajectory_msg(group_name, self.stored_plans[group_name])             
                N = len(jt.goal.trajectory.points)
                rospy.logwarn(str("executing path of " + str(N) + " points"))
                self.command_topics[group_name].publish(jt)
                self.plan_generated[group_name] = False
                r = True # no better way for monitoring success here as it is just an open-loop way of publishing the path
        else :
            rospy.logwarn(str("MoveItPathPlanner::execute_plan() -- no plan for group " + group_name + " yet generated."))
            r = False
        rospy.logdebug(str("MoveItPathPlanner::execute_plan() -- plan execution: " + str(r)))
        return r


    def go(self, group_name, wait=False) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::go() -- group name \'" + str(group_name) + "\' not found"))
            return False
        else :
            return self.groups[group_name].go(wait)

    


    ##################################
    ######## PLANNING METHODS ########
    ##################################    
    

    def plan_joint_goals(self, group_names, goals) :
        
        rospy.loginfo("AtlasPathPlanner::plan_joint_goals()")

        traj_results = {}
        for group_name in group_names :
            traj_results[group_name] = None

        if not len(group_names) == len(goals) :
            rospy.logerr(str("MoveItPathPlanner::plan_joint_goals() -- size mismatch: " 
                + str(len(group_names)) + " groups vs. " + str(len(goals)) + " goals"))
            return traj_results 

        idx = 0
        for group_name in group_names :
            try :
                self.groups[group_name].set_joint_value_target(goals[idx])       
                plan = self.groups[group_name].plan()
                traj_results[group_name] = plan.joint_trajectory
            except :
                rospy.logwarn(str("MoveItPathPlanner::plan_joint_goals(" + group_name + ") -- failed"))                
            idx += 1

        return traj_results


    def plan_joint_paths(self, group_names, paths) :
        rospy.logerror("MoveItPathPlanner::plan_joint_paths() -- not implemented")
        raise NotImplementedError


    def plan_cartesian_goals(self, group_names, goals) :

        traj_results = {}
        for group_name in group_names :
            traj_results[group_name] = None

        if not len(group_names) == len(goals) :
            rospy.logerr(str("MoveItPathPlanner::plan_cartesian_goals() -- size mismatch: " 
                + str(len(group_names)) + " groups vs. " + str(len(goals)) + " goals"))
            return traj_results 

        idx = 0
        for group_name in group_names :
            if not group_name in self.groups.keys() :
                rospy.logerr(str("MoveItPathPlanner::plan_cartesian_goals() -- group name \'" + str(group_name) + "\' not found"))
            else :
                try :
                    self.groups[group_name].set_pose_target(goals[idx])       
                    plan = self.groups[group_name].plan()
                    # if self.auto_execute[group_name] :
                    #     self.execute(group_name, from_stored=True)
                    traj_results[group_name] = plan.joint_trajectory
                except :
                    rospy.logwarn(str("MoveItPathPlanner::plan_to_cartesian_point(" + group_name + ") -- failed"))
            idx += 1

        return traj_results

    def plan_cartesian_paths(self, group_names, paths) :

        traj_results = {}
        for group_name in group_names :
            traj_results[group_name] = None
      
        if not len(group_names) == len(paths) :
            rospy.logerr(str("MoveItPathPlanner::plan_cartesian_paths() -- size mismatch: " 
                + str(len(group_names)) + " groups vs. " + str(len(paths)) + " paths"))
            return traj_results 

        idx = 0
        for group_name in group_names :
            if not group_name in self.groups.keys() :
                rospy.logerr(str("MoveItPathPlanner::plan_cartesian_goals() -- group name \'" + str(group_name) + "\' not found"))
            else :
                try :
                    fraction = 0
                    try :
                        stripped_path = []
                        for p in paths[idx] :
                            stripped_path.append(p.pose)
                        # this jump parameter often makes it fail---more investigation needed here.
                        # also, this will create Cartesian trajectories at a 1cm resolution through all the input waypoints.
                        (plan, fraction) = self.groups[group_name].compute_cartesian_path(stripped_path, 0.01, 0)  
                        if fraction < 0 :
                            rospy.logwarn(str("MoveItPathPlanner::plan_cartesian_paths(" + group_name + ") -- failed, fraction: " + str(fraction)))
                        else :
                            traj_results[group_name] =  plan.joint_trajectory            
                    except :
                        rospy.logerr("MoveItInterface::plan_cartesian_paths() -- Generating Cartesian Path Plan Failed")
                except :
                    rospy.logwarn(str("MoveItPathPlanner::plan_cartesian_paths(" + group_name + ") -- failed"))

        print "traj_results"
        print traj_results            
        return traj_results

                
    def clear_goal_targets(self, group_names) :
        for g in group_names :
            self.clear_goal_target(g)
        

    # There is the option of publishing the JT topic directly to the actionlib goal topic, or to the "bridge" topic
    # that is necessary when commanding a robot using ROS groovy (the JT msg structure changed in hydro -- thanks osrf)  
    def translate_trajectory_msg(self, group_name, jt_in) :
        if group_name in self.bridge_topic_map : 
            rospy.logdebug(str("MoveItPathPlanner::translate_trajectory_msg() -- publishing to bridge topic: " + group_name))
            jt = self.translate_to_bridge_msg(jt_in)
        else :
            rospy.logdebug("MoveItPathPlanner::translate_trajectory_msg() -- publishing to actionlib goal topic")
            jt = self.translate_to_actionlib_goal_msg(jt_in, group_name)
        return jt

    # this takes the JointTrajectory msg and converts it to a JointTrajectoryBridge msg 
    # (necessary when sending to pre-hydro ROS systems)
    def translate_to_bridge_msg(self, jt) :
        from pr2_joint_trajectory_bridge.msg import JointTrajectoryBridge, JointTrajectoryPointBridge
        jt_old = JointTrajectoryBridge()
        jt_old.header = jt.header
        jt_old.joint_names = jt.joint_names
        jt_old.points = []
        for p in jt.points :
            point = JointTrajectoryPointBridge()
            point.positions = p.positions
            point.velocities = p.velocities
            point.accelerations = p.accelerations
            point.time_from_start = p.time_from_start
            jt_old.points.append(point)
        return jt_old

    # this takes the JointTrajectory msg and converts it to a FollowJointTrajectoryActionGoal msg
    def translate_to_actionlib_goal_msg(self, jt, group_name, duration=2.0) :
        from control_msgs.msg import FollowJointTrajectoryActionGoal
        from actionlib_msgs.msg import GoalID
        t = rospy.Time.now()
        ag = FollowJointTrajectoryActionGoal()
        ag.header.stamp = t
        ag.goal_id.stamp = t
        ag.goal_id.id = str("action_goal[" + group_name + "]_" + str(int(random.random()*10000000)))
        ag.goal.trajectory = jt
        ag.goal.goal_time_tolerance = rospy.Duration(duration)       
        return ag

    def has_joint_map(self, group_name) :
        return False

