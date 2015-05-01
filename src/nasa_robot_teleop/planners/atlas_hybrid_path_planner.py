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
import control_msgs.msg

from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *

from nasa_robot_teleop.planners.moveit_path_planner import MoveItPathPlanner
from nasa_robot_teleop.planners.atlas_path_planner import AtlasPathPlanner

class AtlasHybridPathPlanner(PathPlanner) :

    ############################
    ####### CONSTRUCTOR ########   
    ############################

    def __init__(self, robot_name, config_file):
        PathPlanner.__init__(self, robot_name, config_file)

        self.config_file = config_file

        self.moveit_path_planner = MoveItPathPlanner(robot_name, config_file)
        self.atlas_path_planner = AtlasPathPlanner(robot_name, config_file)

        self.manipulation_mode =  rospy.get_param("~atlas/manipulation_mode", "moveit")

        if self.manipulation_mode == "atlas" :
            self.manipulation_planner = self.atlas_path_planner
        else :
            self.manipulation_planner = self.moveit_path_planner

    def setup_group(self, group_name, joint_tolerance, position_tolerances, orientation_tolerances) :
        return self.manipulation_planner.setup_group(group_name, joint_tolerance, position_tolerances, orientation_tolerances)

    def add_obstacle(self, p, s, n) :
        self.manipulation_planner.add_obstacle(p,s,n)

    def get_group_planning_frame(self, group_name) :
        return self.manipulation_planner.get_group_planning_frame(group_name)

    def get_robot_planning_frame(self) :
        return self.manipulation_planner.get_robot_planning_frame()

    def get_group_joints(self, group_name) :
        return self.manipulation_planner.get_group_joints(group_name)

    def has_end_effector_link(self, group_name) :
        return self.manipulation_planner.has_end_effector_link(group_name)

    def get_end_effector_link(self, group_name) :
        return self.manipulation_planner.get_end_effector_link(group_name)

    def has_joint_map(self, group_name) :
        return self.manipulation_planner.has_joint_map(group_name)

    def get_joint_map(self, group_name) :
        return self.manipulation_planner.get_joint_map(group_name)
  
    def get_feet_names(self) :
        return self.atlas_path_planner.get_feet_names()

    def get_start_foot(self) :
        return self.atlas_path_planner.get_start_foot()

    def set_start_foot(self, foot) :
        self.atlas_path_planner.set_start_foot(foot)

    def get_foot_display_pose_offset(self, foot_name) :
        self.atlas_path_planner.get_foot_display_pose_offset(foot_name)

    def clear_goal_targets(self, group_names) :
        self.manipulation_planner.clear_goal_targets(group_names)

    def clear_goal_target(self, group_names) :
        self.manipulation_planner.clear_goal_target(group_names)

    def get_navigation_modes(self) :
        return self.atlas_path_planner.get_navigation_modes()

    def get_navigation_mode(self) :
        return self.atlas_path_planner.get_navigation_mode()

    def set_navigation_mode(self, mode) :
        return self.atlas_path_planner.set_navigation_mode(mode)

    def accommodate_terrain_in_navigation(self) :
        return self.atlas_path_planner.accommodate_terrain_in_navigation()

    def set_accommodate_terrain_in_navigation(self, val) :
        self.atlas_path_planner.set_accommodate_terrain_in_navigation(val)

    #### NAVIGATION FUNCTIONS  
    def plan_navigation_path(self, waypoints) :
        return self.atlas_path_planner.plan_navigation_path(waypoints)


    #### CARTESIAN FUNCTIONS
    def plan_cartesian_goals(self, group_names, goals) :
        return self.manipulation_planner.plan_cartesian_goals(group_names, goals)

    def plan_cartesian_paths(self, group_names, paths) :
        return self.manipulation_planner.plan_cartesian_paths(group_names, paths)


    #### JOINT FUNCTIONS
    def plan_joint_goals(self, group_names, goals) :
        return self.manipulation_planner.plan_joint_goals(group_names, goals)

    def plan_joint_paths(self, group_names) :
        return self.manipulation_planner.plan_joint_paths(group_names)
    

    #### EXCECUTION FUNCTIONS
    def execute_navigation_plan(self, footsteps, lift_heights, feet, goals) :
        return self.atlas_path_planner.execute_navigation_plan(footsteps, lift_heights, feet, goals)

    def execute_plans(self, group_names, from_stored, wait) :
        return self.manipulation_planner.execute_plans(group_names, from_stored, wait)

    def direct_move(self, goal) :
        return self.atlas_path_planner.direct_move(goal)


    def is_plan_generated(self, group) :
        return self.manipulation_planner.is_plan_generated(group)
 