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
        self.use_tolerances = True
        self.groups = {}
              
        self.position_tolerance_modes = {}
        self.orientation_tolerance_modes = {}
        self.max_position_tolerance_modes = {}
        self.max_orientation_tolerance_modes = {}
        
        rospy.set_param("/atlas_path_planner/interpolation_type", 1)
        rospy.set_param("/atlas_path_planner/duration", 2.0)
        rospy.set_param("/atlas_path_planner/num_visualizaton_points", 5)
        rospy.set_param("/atlas_path_planner/visualize_path", True)
        rospy.set_param("/atlas_path_planner/maintain_hand_pose_offsets", True)
        rospy.set_param("/atlas_path_planner/move_as_far_as_possible", True)
            
        self.set_tolerance_file(str(RosPack().get_path('nasa_robot_teleop') + "/config/tolerances.yaml"))

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
            
            if self.use_tolerances :
                self.position_tolerance_modes[group_name] = "FULL"
                self.orientation_tolerance_modes[group_name] = "FULL"
                
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
        # print "trying to set goal pos tol: ", group_name, " to ", tol
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
        # print "trying to set goal angle tol: ", group_name, " to ", tol
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::set_goal_orientation_tolerances() -- group name \'" + str(group_name) + "\' not found"))
        elif len(tol) != 3 :
            rospy.logerr(str("AtlasPathPlanner::set_goal_orientation_tolerances() -- tol vector not of size 3"))
        else:
            m = self.tolerances.get_tolerance_mode('OrientationTolerance', [tol[0],tol[1],tol[2]])
            if m :
                self.orientation_tolerance_modes[group_name] = m
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
    
    def execute(self, group_name, from_stored=False, wait=True) :
        rospy.loginfo(str("AtlasPathPlanner::execute(" + group_name+ ")"))
        rospy.wait_for_service("/atlas_planner/execute_command")
        try :
            executor = rospy.ServiceProxy("/atlas_planner/execute_command", ExecuteCommand)
            resp = executor()
            for p in resp.progress :
                rospy.loginfo(str("AtlasPathPlanner::execute(" + group_name + ") progress: " + str(p)))
            return True
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::execute(" + group_name + ") -- ExecuteCommand service call failed: " + str(e)))
            return False


    def multigroup_execute(self, group_names, from_stored=False, wait=True) :
        r = []
        for g in group_names:
            r.append(self.execute(g,from_stored,wait))
        return r


    ##################################
    ######## PLANNING METHODS ########
    ##################################    
    
    def plan_to_cartesian_goal(self, group_name, pt) :
        req = PlanCommandRequest()
        
        interpolation_type = rospy.get_param("/atlas_path_planner/interpolation_type")
        duration = rospy.get_param("/atlas_path_planner/duration")
        num_visualizaton_points = rospy.get_param("/atlas_path_planner/num_visualizaton_points")

        req.return_trajectories = rospy.get_param("/atlas_path_planner/visualize_path")
        req.maintain_hand_pose_offsets = rospy.get_param("/atlas_path_planner/maintain_hand_pose_offsets")
        req.move_as_far_as_possible = rospy.get_param("/atlas_path_planner/move_as_far_as_possible")

        if group_name in self.groups.keys() :
            
            req.execute_on_plan = self.auto_execute[group_name]
    
            spec = PlanRequestSpecification()
            spec.group_id = self.groups[group_name].group_id
            spec.group_name = self.groups[group_name].group_name
            spec.joint_mask = self.groups[group_name].joint_mask
            spec.control_frame = self.groups[group_name].control_frame
            spec.waypoints.append(pt)
            spec.duration.append(duration)
            spec.num_visualizaton_points = num_visualizaton_points
            spec.angle_variance.append(self.orientation_tolerance_modes[group_name]) 
            spec.maximum_angle_variance.append("") 
            spec.position_variance.append(self.position_tolerance_modes[group_name]) 
            spec.maximum_position_variance.append("") 
            spec.interpolation_type = interpolation_type
            req.group_plan_specs.append(spec)

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
        rospy.wait_for_service("/atlas_planner/joint_plan_command")
        try :
            planner = rospy.ServiceProxy("/atlas_planner/joint_plan_command", PlanCommand)
            resp = planner(req)
            return resp.result
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::plan_to_joint_goal(" + group_name + ") -- JointPlanCommand service call failed: " + str(e)))
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


        interpolation_type = rospy.get_param("/atlas_path_planner/interpolation_type")
        duration = rospy.get_param("/atlas_path_planner/duration")
        num_visualizaton_points = rospy.get_param("/atlas_path_planner/num_visualizaton_points")

        req.return_trajectories = rospy.get_param("/atlas_path_planner/visualize_path")
        req.maintain_hand_pose_offsets = rospy.get_param("/atlas_path_planner/maintain_hand_pose_offsets")
        req.move_as_far_as_possible = rospy.get_param("/atlas_path_planner/move_as_far_as_possible")

        if group_name in self.groups.keys() :

            req.execute_on_plan = self.auto_execute[group_name]

            spec = PlanRequestSpecification()
            spec.group_id = self.groups[group_name].group_id
            spec.group_name = self.groups[group_name].group_name
            spec.joint_mask = self.groups[group_name].joint_mask
            spec.control_frame = self.groups[group_name].control_frame
        
            for wp in waypoints :
                spec.waypoints.append(wp)
                spec.duration.append(duration)
                spec.angle_variance.append(self.orientation_tolerance_modes[group_name]) 
                spec.maximum_angle_variance.append("") 
                spec.position_variance.append(self.position_tolerance_modes[group_name]) 
                spec.maximum_position_variance.append("") 
            
            spec.interpolation_type = interpolation_type
            spec.num_visualizaton_points = num_visualizaton_points
            req.group_plan_specs.append(spec)

        else :
            rospy.logerr(str("AtlasPathPlanner::plan_cartesian_pathplan_to_cartesian_goal(" + group_name + ") -- no group found of that name!"))
            return False

        rospy.wait_for_service("/atlas_planner/plan_command")
        try :
            planner = rospy.ServiceProxy("/atlas_planner/plan_command", PlanCommand)
            resp = planner(req)
            return resp.result
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- PlanCommand service call failed: " + str(e)))
            return False


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
    
    def has_joint_map(self, group_name) :
        return True

    def get_joint_map(self, group_name) :
        return self.groups[group_name].joint_map

    def has_joint_mask(self, group_name) :
        return True

    def get_joint_mask(self, group_name) :
        return self.groups[group_name].joint_mask.mask
        
    def set_joint_mask(self, group_name, mask) :
        self.groups[group_name].joint_mask.mask = mask

if __name__=="__main__":

    rospy.init_node("atlas_planner_client")
    pp = AtlasPathPlanner("atlas", "atlas_moveit_config")   
    rospy.spin()