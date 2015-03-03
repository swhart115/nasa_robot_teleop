#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg

from path_planner import *
from tolerances import *

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *

# 1. use init() to setup and store group information by making GetPlanningServiceConfiguration.srv call
# 2. deal with execution and planning functions

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
              
        self.position_tolerance_modes = {}
        self.orientation_tolerance_modes = {}
        self.max_position_tolerance_modes = {}
        self.max_orientation_tolerance_modes = {}
        
        self.load_configurations()       
        rospy.loginfo(str("============ Setting up Path Planner for robot: \'" + self.robot_name + "\' finished"))
        

    ##############################
    ####### SETUP METHODS ########   
    ##############################

    def load_configurations(self) :
        req = GetPlanningServiceConfigurationRequest()
        rospy.wait_for_service("/atlas_planner/get_config")
        try :
            get_planner_config = rospy.ServiceProxy("/atlas_planner/get_config", GetPlanningServiceConfiguration)
            resp = get_planner_config(req)
            for g in resp.group_configurations:
                self.groups[g.group_name] = g
            print resp
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::load_configurations() -- GetPlanningServiceConfiguration service call failed: " + str(e)))

    def configure_group(self, group_name) :
        r = True
        rospy.loginfo(str("AtlasPathPlanner::configure_group() -- " + group_name))     
        if not group_name in self.groups :
            rospy.logerr(str("AtlasPathPlanner::configure_group(" + group_name + ") -- group not found"))
            return False
        req = ConfigurePlanningServiceRequest()
        rospy.wait_for_service("/atlas_planner/config")
        try :
            req.group_configurations.append(self.groups[group_name])
            configure_planner = rospy.ServiceProxy("/atlas_planner/config", ConfigurePlanningService)
            resp = configure_planner(req)
            if not resp.status :
                rospy.logwarn(str("AtlasPathPlanner::configure_group(" + group_name + ") -- status error"))
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::configure_group() -- Service call failed: " + str(e)))
            r = False
        return r 

    def setup_group(self, group_name, joint_tolerance, position_tolerance, orientation_tolerance) :
        rospy.loginfo(str("AtlasPathPlanner::setup_group() -- " + group_name))     
        
        if group_name in self.groups.keys() :
            
            rospy.loginfo(str("AtlasPathPlanner::setup_group() -- " + group_name))
            
            m = self.tolerances.get_tolerance_mode('PositionTolerance', [position_tolerance[0],position_tolerance[1],position_tolerance[2]])
            if m: self.position_tolerance_modes[group_name] = m            
            
            m = self.tolerances.get_tolerance_mode('OrientationTolerance', [orientation_tolerance[0],orientation_tolerance[1],orientation_tolerance[2]])
            if m :self.orientation_tolerance_modes[group_name]
        
            return True
        else : 
            return False

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
        pass

    def get_group_joints(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_group_joints() -- group name \'" + str(group_name) + "\' not found"))
            return []
        else :
            return self.groups[group_name].joint_map.names
        
    def get_goal_position_tolerances(self, group_name) :
        if not group_name in self.position_tolerance_modes.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_goal_position_tolerances() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.tolerances.get_tolerance_vals('PositionTolerance', self.position_tolerance_modes[group_name])

    def get_goal_orientation_tolerances(self, group_name) :
        if not group_name in self.orientation_tolerance_modes.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_goal_orientation_tolerances() -- group name \'" + str(group_name) + "\' not found"))
            return 0
        else :
            return self.tolerances.get_tolerance_vals('OrientationTolerance', self.orientation_tolerance_modes[group_name])
    
    def get_goal_joint_tolerance(self, group_name) :
        rospy.logerr(str("AtlasPathPlanner::get_goal_joint_tolerance() -- not implemented"))
        
        # if not group_name in self.groups.keys() :
        #     rospy.logerr(str("AtlasPathPlanner::get_goal_joint_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        #     return 0
        # else :
        #     return self.groups[group_name].get_goal_joint_tolerance() # FIXME

    def set_goal_tolerance(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        else :
            m = PositionTolerance.get_tolerance_mode(tol,tol,tol)
            if m: 
                self.position_tolerance_modes[group_name] = m            
            else :
                rospy.logwarn(str("AtlasPathPlanner::set_goal_tolerance() -- NO POSITION TOLERANCE MODE SET"))

            m = self.tolerances.get_tolerance_mode('OrientationTolerance', [tol,tol,tol])
            if m: 
                self.orientation_tolerance_modes[group_name] = m            
            else :
                rospy.logwarn(str("AtlasPathPlanner::set_goal_tolerance() -- NO ORIENTATION TOLERANCE MODE SET"))

    def set_goal_position_tolerances(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_position_tolerances() -- group name \'" + str(group_name) + "\' not found"))
        elif len(tol) != 3 :
            rospy.logerr(str("AtlasPathPlanner::set_goal_position_tolerances() -- tol vector not of size 3"))
        else:
            m = self.tolerances.get_tolerance_mode('PositionTolerance', [tol[0],tol[1],tol[2]])
            if m: 
                self.position_tolerance_modes[group_name] = m            
            else :
                rospy.logwarn(str("AtlasPathPlanner::set_goal_position_tolerances() -- NO TOLERANCE MODE SET"))

    def set_goal_orientation_tolerances(self, group_name, tol) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_orientation_tolerances() -- group name \'" + str(group_name) + "\' not found"))
        elif len(tol) != 3 :
            rospy.logerr(str("AtlasPathPlanner::set_goal_orientation_tolerances() -- tol vector not of size 3"))
        else:
            m = OrientationTolerance.get_tolerance_mode(tol[0],tol[1],tol[2])
            if m :
                self.orientation_tolerance_modes[group_name]
            else :
                rospy.logwarn(str("AtlasPathPlanner::set_goal_orientation_tolerances() -- NO TOLERANCE MODE SET"))
        
    def set_goal_joint_tolerance(self, group_name, tol) :
        rospy.logerr(str("AtlasPathPlanner::set_goal_joint_tolerance() -- not implemented"))

        # if not group_name in self.groups.keys() :
        #     rospy.logerr(str("AtlasPathPlanner::set_goal_joint_tolerance() -- group name \'" + str(group_name) + "\' not found"))
        # else :
        #     self.groups[group_name].set_goal_joint_tolerance(tol) # FIXME

    ###################################
    ######## EXECUTION METHODS ########
    ###################################
    
    def go(self, group_name, wait=False) :
        rospy.wait_for_service("/atlas_planner/execute_command")
        try :
            executor = rospy.ServiceProxy("/atlas_planner/execute_command", ExecuteCommand)
            resp = executor()
            print "AtlasPathPlanner::go(" + group_name + ") progress: " + resp.progress
            return True
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::go(" + group_name + ") -- ExecuteCommand service call failed: " + str(e)))
            return None

    def multigroup_go(self, group_names, wait) :
        r = []
        for g in group_names:
            r.append(self.go(g,wait))
        return r


    ##################################
    ######## PLANNING METHODS ########
    ##################################    
    
    def plan_to_cartesian_goal(self, group_name, pt) :
        req = PlanCommandRequest()
        
        if group_name in self.groups.keys() :
            spec = copy.deepcopy(self.groups[group_name])
            spec.waypoints.append(pt)
            req.group_plan_specs.append(spec)

            req.move_as_far_as_possible  = True
            req.maintain_hand_pose_offsets = False
            req.execute_on_plan = False
            req.return_trajectories = True

        else :
            rospy.logerr(str("AtlasPathPlanner::plan_cartesian_pathplan_to_cartesian_goal(" + group_name + ") -- no group found of that name!"))
            return None

        rospy.wait_for_service("/atlas_planner/plan_command")
        try :
            planner = rospy.ServiceProxy("/atlas_planner/plan_command", PlanCommand)
            resp = planner(req)
            return resp.result
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- PlanCommand service call failed: " + str(e)))
            return None

    def plan_to_joint_goal(self, group_name, js) :
        try :
            self.groups[group_name].set_joint_value_target(js)       
            plan = self.groups[group_name].plan()               # FIXME
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("AtlasPathPlanner::plan_to_joint_goal(" + group_name + ") -- failed"))
            return None

    def plan_to_random_goal(self, group_name) :
        try :
            self.groups[group_name].set_random_target() # FIXME
            plan = self.groups[group_name].plan()  # FIXME
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("AtlasPathPlanner::plan_to_random_goal(" + group_name + ") -- failed"))
            return None

    def plan_cartesian_path(self, group_name, waypoints) :
        req = PlanCommandRequest()
        rospy.wait_for_service("/atlas_planner/plan_command")
        
        if group_name in self.groups.keys() :
            spec = copy.deepcopy(self.groups[group_name])
            spec.waypoints = copy.deepcopy(waypoints)
            req.group_plan_specs.append(spec)

            req.move_as_far_as_possible  = True
            req.maintain_hand_pose_offsets = False
            req.execute_on_plan = False
            # req.execute_previous_plan
            req.return_trajectories = True
        else :
            rospy.logerr(str("AtlasPathPlanner::plan_cartesian_path(" + group_name + ") -- no group found of that name!"))
            return None

        try :
            planner = rospy.ServiceProxy("/atlas_planner/plan_command", PlanCommand)
            resp = planner(req)
            return resp.result
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::plan_cartesian_path(" + group_name + ") -- PlanCommand service call failed: " + str(e)))
            return None


    ### multigroup functions
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
        


if __name__=="__main__":

    rospy.init_node("atlas_planner_client")
    pp = AtlasPathPlanner("atlas", "atlas_moveit_config")   
    rospy.spin()