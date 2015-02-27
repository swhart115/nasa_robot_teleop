#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import drc_control.CartesianR

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg

from path_planner import *

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *


# 1. use init() to setup and store group information (possibly by making GetPlanningServiceConfiguration.srv calls)
# 2. use setup_group() to take that set-up information and call the ConfigurePlanningService.srv request for the specified group
# 3. deal with execution and planning functions

# - Add a setup_groups(group_names[] function)??
# - setup groups in init() and just verify in setup_group() that it exists?
# - joint planning should allow us to use masks and set individual joint goals

class AtlasPathPlanner(PathPlanner) :

    ############################
    ####### CONSTRUCTOR ########   
    ############################

    def __init__(self, robot_name, config_package):
        PathPlanner.__init__(self, robot_name, config_package)
        rospy.loginfo(str("============ Setting up Path Planner for: \'" + self.robot_name + "\'"))
        self.planning_frame = "/world"
        self.groups = {}
        self.load_configurations()
        rospy.loginfo(str("============ Setting up Path Planner for robot: \'" + self.robot_name + "\' finished"))
        
    ##############################
    ####### SETUP METHODS ########   
    ##############################

    def load_configurations(self) :
        print "what to do here....."

    def setup_group(self, group_name, joint_tolerance, position_tolerance, orientation_tolerance) :
        r = True
        rospy.loginfo(str("AtlasPathPlanner::setup_group() -- " + group_name))     
        if not group_name in self.groups :
            rospy.logerr(str("AtlasPathPlanner::setup_group(" + group_name + ") -- group not found"))
            return False
        req = ConfigurePlanningServiceRequest()
        rospy.wait_for_service("/atlas_planner/configure")
        try :
            req.group_configurations.append(self.groups[group_name])
            configure_planner = rospy.ServiceProxy("/atlas_planner/configure", ConfigurePlanningService)
            resp = configure_planner(req)
            if not resp.status :
                rospy.logwarn(str("AtlasPathPlanner::setup_group(" + group_name + ") -- status error"))
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::init() -- Service call failed: " + str(e))
            r = False
        return r 

    #################################
    ####### OBSTACLE METHODS ########   
    #################################

    def add_obstacle(self, p, s, n) :
        rospy.logerr("AtlasPathPlanner::add_obstacle() -- obstacle avoidance not supported")
       

    ################################
    ######## HELPER METHODS ########
    ################################
    
    def get_robot_planning_frame(self) :
        return self.planning_frame

    def get_group_planning_frame(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_group_planning_frame() -- group name \'" + str(group_name) + "\' not found"))
            return ""
        else :
            return self.groups[group_name].planning_frame

    def has_end_effector_link(self, group_name) :
        # print self.groups.keys()
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::has_end_effector_link() -- group name \'" + str(group_name) + "\' not found"))
            return False
        else :
            return self.groups[group_name].control_frame != None

    def get_end_effector_link(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_end_effector_link() -- group name \'" + str(group_name) + "\' not found"))
            return ""
        else :
            return self.groups[group_name].control_frame

    def clear_goal_target(self, group_name) :
        try :
            self.groups[group_name].clear_pose_targets()  # FIXME
        except :
            rospy.logwarn(str("AtlasPathPlanner::clear_goal_target(" + group_name + ") -- failed"))

    def get_group_joints(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_group_joints() -- group name \'" + str(group_name) + "\' not found"))
            return [] 
        else :
            self.groups[group_name].joint_map.names
        
    def get_goal_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_goal_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_tolerance()  # FIXME

    def get_goal_position_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_goal_position_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_position_tolerance() # FIXME

    def get_goal_joint_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_goal_joint_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_joint_tolerance() # FIXME

    def get_goal_orientation_tolerance(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_goal_orientation_tolerance() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.groups[group_name].get_goal_orientation_tolerance() # FIXME
    
    def set_goal_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            self.groups[group_name].set_goal_tolerance(tol) # FIXME

    def set_goal_position_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_position_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            self.groups[group_name].set_goal_position_tolerance(tol) # FIXME

    def set_goal_joint_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_joint_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            self.groups[group_name].set_goal_joint_tolerance(tol) # FIXME

    def set_goal_orientation_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_orientation_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            self.groups[group_name].set_goal_orientation_tolerance(tol) # FIXME


    ###################################
    ######## EXECUTION METHODS ########
    ###################################
    
    def go(self, group_name, wait=False) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::go() -- group name \'" + str(group_name) + "\' not found"))
            return False
        else :
            return self.groups[group_name].go(wait)

    def multigroup_go(self, group_names, wait) :
        r = []
        for g in group_names:
            r.append(self.go(g,wait))
        return r


    ##################################
    ######## PLANNING METHODS ########
    ##################################    
    
    def plan_to_cartesian_goal(self, group_name, pt) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::go() -- group name \'" + str(group_name) + "\' not found"))
        else :
            try :
                self.groups[group_name].set_pose_target(pt)       
                plan = self.groups[group_name].plan()
                return plan.joint_trajectory
            except :
                rospy.logwarn(str("AtlasPathPlanner::plan_to_cartesian_point(" + group_name + ") -- failed"))
                return None

    def plan_to_joint_goal(self, group_name, js) :
        try :
            self.groups[group_name].set_joint_value_target(js)       
            plan = self.groups[group_name].plan()
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("AtlasPathPlanner::plan_to_joint_goal(" + group_name + ") -- failed"))
            return None

    def plan_to_random_goal(self, group_name) :
        try :
            self.groups[group_name].set_random_target()
            plan = self.groups[group_name].plan()
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("AtlasPathPlanner::plan_to_random_goal(" + group_name + ") -- failed"))
            return None

    def plan_cartesian_path(self, group_name, waypoints) :
        try :
            fraction = 0
            self.groups[group_name].compute_cartesian_path(waypoints, 0.01, 0)  
            (plan, fraction) = self.groups[group_name].plan()
            if fraction < 0 :
                rospy.logwarn(str("AtlasPathPlanner::plan_cartesian_path(" + group_name + ") -- failed, fraction: " + str(fraction)))
                return None
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("AtlasPathPlanner::plan_cartesian_path(" + group_name + ") -- failed"))
            return None


    def plan_to_cartesian_goals(self, group_names, pts) :
        r = []
        if not len(group_names) == len(pts) :
            rospy.logerr("AtlasPathPlanner::plan_to_cartesian_goals() -- input arg size mismatch")
            r.append(False)
        else :
            for i in len(group_names) :
                r.append(self.plan_to_cartesian_goal(group_names[i], pts[i]))
        return r

    def plan_to_joint_goals(self, group_names, jss) :
        r = []
        if not len(group_names) == len(jss) :
            rospy.logerr("AtlasPathPlanner::plan_to_joint_goals() -- input arg size mismatch")
            r.append(False)
        else :
            for i in len(group_names) :
                r.append(self.plan_to_joint_goal(group_names[i], jss[i]))
        return r
        
    def plan_to_random_goals(self, group_names) :
        r = []
        for i in len(group_names) :
            r.append(self.plan_to_random_goal(group_names[i]))
        return r
        
    def plan_cartesian_paths(self, group_names, frame_ids, pt_lists) :
        r = []
        if not len(group_names) == len(pt_lists) == len(frame_ids):
            rospy.logerr("AtlasPathPlanner::plan_cartesian_paths() -- input arg size mismatch")
            r.append(False)
        else :
            for i in len(group_names) :
                r.append(self.plan_cartesian_path(group_names[i],frame_ids[i], pt_lists[i]))
        return r
                
    def clear_goal_targets(self, group_names) :
        for g in group_names :
            self.clear_goal_target(g)
        