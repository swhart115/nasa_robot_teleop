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

# import controller_manager_msgs.srv
# from nasa_robot_teleop.srv import *

from path_planner import *


class MoveItPathPlanner(PathPlanner) :

    ############################
    ####### CONSTRUCTOR ########   
    ############################

    def __init__(self, robot_name, config_package):
        PathPlanner.__init__(self, robot_name, config_package)

        # connect to moveit server
        rospy.loginfo(str("============ Setting up MoveIt! for robot: \'" + self.robot_name + "\'"))
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.groups = {}
        
        # set up obstacle publisher stuff
        self.obstacle_markers = visualization_msgs.msg.MarkerArray()
        self.obstacle_publisher = rospy.Publisher(str('/' + self.robot_name + '/obstacle_markers'), visualization_msgs.msg.MarkerArray,queue_size=10)
        
        rospy.loginfo(str("============ Setting up MoveIt! for robot: \'" + self.robot_name + "\' finished"))
        
    ##############################
    ####### SETUP METHODS ########   
    ##############################

    def setup_group(self, group_name, joint_tolerance, position_tolerance, orientation_tolerance) :
        r = True
        rospy.loginfo(str("MoveItPathPlanner::setup_group() -- " + group_name))     
        try :
            self.groups[group_name] = moveit_commander.MoveGroupCommander(group_name)
            self.groups[group_name].set_goal_joint_tolerance(joint_tolerance)
            self.groups[group_name].set_goal_position_tolerance(position_tolerance)
            self.groups[group_name].set_goal_orientation_tolerance(orientation_tolerance)
        except :
            rospy.logerr(str("MoveItInterface()::setup_group() -- Robot " + self.robot_name + " has problem setting up MoveIt! commander group for: " + group_name))
            r = False
        return r 

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
        # print self.groups.keys()
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

    def clear_goal_targets(self, group_name) :
        try :
            self.groups[group_name].clear_pose_targets()
        except :
            rospy.logwarn(str("MoveItPathPlanner::clear_goal_targets(" + group_name + ") -- failed"))

    def get_group_joints(self, group_name) :
        if self.robot :
            return self.robot.get_joint_names(group_name)
        else :
            rospy.logerr(str("MoveItPathPlanner::get_group_joints() -- group name \'" + str(group_name) + "\' not found"))
            return [] 

    def get_goal_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::get_goal_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_tolerance()

    def get_goal_position_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::get_goal_position_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_position_tolerance()

    def get_goal_joint_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::get_goal_joint_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_joint_tolerance()

    def get_goal_orientation_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::get_goal_orientation_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_orientation_tolerance()
    
    def set_goal_position_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::set_goal_position_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            self.groups[group_name].set_goal_position_tolerance(tol)

    def set_goal_joint_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::set_goal_joint_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            self.groups[group_name].set_goal_joint_tolerance(tol)

    def set_goal_orientation_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::set_goal_orientation_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            self.groups[group_name].set_goal_orientation_tolerance(tol)


    ###################################
    ######## EXECUTION METHODS ########
    ###################################
    
    def go(self, group_name, wait=False) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::go() -- group name \'" + str(group_name) + "\' not found"))
            return False
        else :
            return self.groups[group_name].go(wait)


    ##################################
    ######## PLANNING METHODS ########
    ##################################    
    
    def plan_to_cartesian_goal(self, group_name, pt) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("MoveItPathPlanner::go() -- group name \'" + str(group_name) + "\' not found"))
        else :
            try :
                self.groups[group_name].set_pose_target(pt)       
                plan = self.groups[group_name].plan()
                return plan.joint_trajectory
            except :
                rospy.logwarn(str("MoveItPathPlanner::plan_to_cartesian_point(" + group_name + ") -- failed"))
                return None

    def plan_to_joint_goal(self, group_name, js) :
        try :
            self.groups[group_name].set_joint_value_target(js)       
            plan = self.groups[group_name].plan()
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("MoveItPathPlanner::plan_to_joint_goal(" + group_name + ") -- failed"))
            return None

    def plan_to_random_goal(self, group_name) :
        try :
            self.groups[group_name].set_random_target()
            plan = self.groups[group_name].plan()
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("MoveItPathPlanner::plan_to_random_goal(" + group_name + ") -- failed"))
            return None

    def plan_cartesian_path(self, group_name, waypoints) :
        try :
            fraction = 0
            self.groups[group_name].compute_cartesian_path(waypoints, 0.01, 0)  
            (plan, fraction) = self.groups[group_name].plan()
            if fraction < 0 :
                rospy.logwarn(str("MoveItPathPlanner::plan_cartesian_path(" + group_name + ") -- failed, fraction: " + str(fraction)))
                return None
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("MoveItPathPlanner::plan_cartesian_path(" + group_name + ") -- failed"))
            return None


