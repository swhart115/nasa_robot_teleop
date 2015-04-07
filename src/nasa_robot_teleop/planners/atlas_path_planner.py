#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import PyKDL as kdl

roslib.load_manifest('step_finder')
roslib.load_manifest('walk_controller')
roslib.load_manifest('matec_msgs')

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg
import std_msgs.msg

import actionlib
from actionlib_msgs.msg import GoalStatus
from matec_msgs.msg import *
from matec_actions.msg import *
from walk_controller.msg import *

from matec_msgs.srv import *
from step_finder.srv import *

from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.util.kinematics_util import *

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *

class AtlasPathPlanner(PathPlanner) :

    ############################
    ####### CONSTRUCTOR ########
    ############################

    def __init__(self, robot_name, config_package):
        PathPlanner.__init__(self, robot_name, config_package)
        rospy.loginfo(str("============ Setting up Path Planner for: \'" + self.robot_name + "\'"))
        self.planning_frame = rospy.get_param("/interactive_control/navigation_frame", "pelvis")
        self.groups = {}
        self.joint_names = []
        self.feet_names = ['left', 'right']
        self.wait_for_service_timeout = 5.0

        rospy.set_param("~interpolation_type", 1)
        rospy.set_param("~num_visualizaton_points", 5)
        rospy.set_param("~visualize_path", True)
        rospy.set_param("~maintain_hand_pose_offsets", False)
        rospy.set_param("~move_as_far_as_possible", False)

        rospy.set_param("~duration", 2.0)
        rospy.set_param("~allow_incomplete_planning", True)
        rospy.set_param("~num_acceptable_consecutive_failures", 0)
        rospy.set_param("~plan_visualization_density", 0.5)
        rospy.set_param("~visualize_on_plan", True)
        rospy.set_param("~max_angular_velocity", 0.4)
        rospy.set_param("~max_linear_velocity", 0.1)

        self.cartesian_reach_client = actionlib.SimpleActionClient('/planned_manipulation/server', matec_actions.msg.PlannedManipulationAction)
        self.joint_action_client = actionlib.SimpleActionClient('/base_joint_interpolator/server', control_msgs.msg.FollowJointTrajectoryAction)
        self.walk_controller_client = actionlib.SimpleActionClient('/path_walker', walk_controller.msg.WalkPathAction)

        self.goal_region_pub = rospy.Publisher(str('/' + self.robot_name + '/planned_path_visualization'), visualization_msgs.msg.MarkerArray, latch=False, queue_size=10)

        # self.planner_feedback_sub = rospy.Subscriber('/planned_manipulation/server/feedback', matec_actions.msg.PlannedManipulationActionFeedback, self.planner_feedback)
        # self.planner_viz_feedback_sub = rospy.Subscriber('/planned_manipulation/plan_visual', control_msgs.msg.FollowJointTrajectoryGoal, self.planner_viz_feedback)
      
        rospy.loginfo("AtlasPathPlanner::init() -- waiting for visualization service")
        self.connected_to_plan_viz = rospy.wait_for_service('/planned_manipulation/visualize', 3.0)
        if not self.connected_to_plan_viz :
            rospy.logwarn("AtlasPathPlanner::init() -- timeout out waitin for visualization service")  

        self.last_plan_name = ""
        self.get_joint_names()

        rospy.loginfo(str("============ Setting up Path Planner for robot: \'" + self.robot_name + "\' finished"))

    ##############################
    ####### SETUP METHODS ########
    ##############################

    def load_configurations(self) :
        req = GetPlanningServiceConfigurationRequest()

        try :
            rospy.wait_for_service("/interactive_controls_bridge/get_config", self.wait_for_service_timeout)
        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::load_configurations(): " + str(e))
            return False

        try :
            get_planner_config = rospy.ServiceProxy("/interactive_controls_bridge/get_config", GetPlanningServiceConfiguration)
            resp = get_planner_config(req)
            for g in resp.group_configurations:
                self.groups[g.group_name] = g
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::load_configurations() -- GetPlanningServiceConfiguration service call failed: " + str(e)))
            return False

        return True

    def configure_group(self, group_name) :
        r = True
        rospy.loginfo(str("AtlasPathPlanner::configure_group() -- " + group_name))
        if not group_name in self.groups :
            rospy.logerr(str("AtlasPathPlanner::configure_group(" + group_name + ") -- group not found"))
            return False
        req = ConfigurePlanningServiceRequest()

        try :
            rospy.wait_for_service("/interactive_controls_bridge/config", self.wait_for_service_timeout)
        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::configure_group(): " + str(e))
            return False
        
        try :
            req.group_configurations.append(self.groups[group_name])
            configure_planner = rospy.ServiceProxy("/interactive_controls_bridge/config", ConfigurePlanningService)
            resp = configure_planner(req)
            if not resp.status :
                rospy.logwarn(str("AtlasPathPlanner::configure_group(" + group_name + ") -- status error"))
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::configure_group() -- Service call failed: " + str(e)))
            r = False
        return r

    def setup_group(self, group_name, joint_tolerance, position_tolerances, orientation_tolerances) :
        rospy.loginfo(str("AtlasPathPlanner::setup_group() -- " + group_name))
        self.position_tolerances[group_name] = position_tolerances
        self.orientation_tolerances[group_name] = orientation_tolerances
        self.joint_tolerance[group_name] = joint_tolerance
        return self.load_group_from_srdf(group_name)

    def load_group_from_srdf(self, group_name) :

        if not group_name in self.srdf_model.groups :
            return False

        rospy.loginfo(str("AtlasPathPlanner::load_group_from_srdf() -- group: " + group_name))

        self.groups[group_name] = PlanGroupConfiguration()
        self.groups[group_name].joint_map = self.lookup_joint_map(group_name)

        N = len(self.groups[group_name].joint_map.names)

        if N==0 :
            rospy.logwarn(str("AtlasPathPlanner::load_group_from_srdf(" + group_name + ") -- no joint names found in the map!"))
            return False

        last_joint = self.groups[group_name].joint_map.names[N-1]

        self.groups[group_name].group_name = group_name
        self.groups[group_name].group_id = self.srdf_model.groups.index(group_name)
        self.groups[group_name].control_frame = self.srdf_model.get_tip_link(group_name)
        self.groups[group_name].planning_frame = self.planning_frame
        self.groups[group_name].joint_mask.mask = self.srdf_model.get_joint_mask(group_name)

        return True

    def lookup_joint_map(self, group_name) :

        root = self.srdf_model.get_base_link(group_name)
        tip = self.srdf_model.get_tip_link(group_name)
        joint_list = []

        if root == tip == '' :
            joint_list = self.srdf_model.get_group_joints(group_name)
        else :
            joint_list = get_chain(self.urdf_model, root, tip, joints=True, fixed=True)

        joint_name_map = JointNameMap()

        for j in joint_list :
            if not j in self.joint_names :
                continue
            joint_name_map.names.append(j)
            joint_name_map.ids.append(self.joint_names.index(j))

        return joint_name_map

    # def joint_name_callback(self, msg) :
    #     self.joint_names = msg.data
    #     rospy.loginfo("GOT JOINT NAMES!!")

    def get_joint_names(self) :
        rospy.loginfo("AtlasPathPlanner::get_joint_names() -- getting joint names from service")

        try : 
            rospy.wait_for_service("/interactive_controls_bridge/get_joint_names", self.wait_for_service_timeout)
        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::get_joint_names(): " + str(e))
            return None
        

        try :
            rospy.loginfo(str("AtlasPathPlanner::get_joint_names() -- calling service"))
            get_names = rospy.ServiceProxy("/interactive_controls_bridge/get_joint_names", GetJointNames)
            resp = get_names()
            self.joint_names = resp.joint_names
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::get_joint_names()" + str(e)))
            return None

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
        if not group_name in self.groups.keys() :
            rospy.loginfo(str("AtlasPathPlanner::has_end_effector_link() -- group name \'" + str(group_name) + "\' not found"))
            return False
        else :
            try :
                if self.group_types[group_name] == "endeffector" :
                    return False
                return (self.srdf_model.get_end_effector_link(group_name) != "")
            except :
                return False

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

    def get_start_foot(self) :
        return rospy.get_param("~start_foot")

    def set_start_foot(self, foot) :
        rospy.set_param("~start_foot", foot)


    def get_foot_display_pose_offset(self, foot_name) :
        p = Pose()
        p.orientation.w=1
        p.position.z = 0.025
        return p

    def get_feet_names(self) :
        return self.feet_names

    ###################################
    ######## EXECUTION METHODS ########
    ###################################

    def execute_plan(self, group_name, from_stored=False, wait=True) :
        rospy.loginfo(str("AtlasPathPlanner::execute_plan(" + group_name + ")"))

        # hack to send individual foot goal to robot
        if "_leg" in group_name :
            rospy.logwarn("AtlasPathPlanner::execute_plan() -- can't execute leg command directly; use \'execute_on_plan\' instead")
            return False

        req = ExecuteManipulationPlanRequest()
        req.plan_names.append(group_name)
        # req.joint_names = self.get_group_joints(group_name)
        try :
            executor = rospy.ServiceProxy("/planned_manipulation/execute", ExecuteManipulationPlan)
            resp = executor(req)
            # for p in resp.progress :
            #     rospy.loginfo(str("AtlasPathPlanner::execute_plan(" + group_name + ") progress: " + str(p)))
            return True
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::execute_plan(" + group_name + ") -- ExecuteCommand service call failed: " + str(e)))
            return False

    def execute_navigation_plan(self, footsteps) :

        rospy.loginfo("AtlasPathPlanner::execute_navigation_plan()")
        req = CartesianPlanCommandRequest()

        interpolation_type = rospy.get_param("~interpolation_type")
        duration = rospy.get_param("~duration")
        num_visualizaton_points = rospy.get_param("~num_visualizaton_points")

        req.return_trajectories = rospy.get_param("~visualize_path")
        req.maintain_hand_pose_offsets = rospy.get_param("~maintain_hand_pose_offsets")
        req.move_as_far_as_possible = rospy.get_param("~move_as_far_as_possible")

        start_foot = self.get_start_foot()
        if start_foot=="left" :
            indicator = 0
        else :
            indicator = 1

        try :
        
            idx = 0
            for foot in footsteps :
                spec = CartesianPlanRequestSpecification()
                
                if idx%2 == indicator :
                    spec.group_name = "left_leg" 
                else :
                    spec.group_name = "right_leg"

                spec.control_frame = foot.header.frame_id

                spec.waypoints.append(foot)
                spec.duration.append(duration)
                # spec.angle_variance.append(0)
                # spec.maximum_angle_variance.append(0)
                # spec.position_variance.append(0)
                # spec.maximum_position_variance.append(0)

                spec.interpolation_type = interpolation_type
                spec.num_visualizaton_points = num_visualizaton_points
                req.group_plan_specs.append(spec)

                idx += 1
        except :
            rospy.logerr(str("AtlasPathPlanner::execute_navigation_plan()"))
            return None

        # print "==================================================="
        # print "Footstep plan:"
        # print req
        # print "==================================================="
        try :
            rospy.wait_for_service("/interactive_controls_bridge/navigation_plan_command", self.wait_for_service_timeout)
        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::execute_navigation_plan(): " + str(e))
            return None

        try :
            rospy.loginfo(str("AtlasPathPlanner::execute_navigation_plan() -- calling service"))
            planner = rospy.ServiceProxy("/interactive_controls_bridge/navigation_plan_command", CartesianPlanCommand)
            resp = planner(req)

            if len(resp.result) > 0 :
                return resp.result[0]
            else :
                rospy.logwarn(str("AtlasPathPlanner::execute_navigation_plan() -- failed to get footstep plan to goal"))
                return None
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::execute_navigation_plan()" + str(e)))
            return None

    def multigroup_execute_plan(self, group_names, from_stored=False, wait=True) :
        r = []
        for g in group_names:
            r.append(self.execute_plan(g,from_stored,wait))
        return r


    def execute_leg_plan(self, group_name, wp) :
        rospy.loginfo("AtlasPathPlanner::execute_leg_plan() -- waiting for walking server")
        self.walk_controller_client.wait_for_server()
        
        # Creates a goal to send to the action server.
        goal = walk_controller.msg.WalkPathGoal()

        goal.left_foot_start == (group_name == "left_leg")

        p = geometry_msgs.msg.Pose2D()
        p.x = wp.pose.position.x
        p.y = wp.pose.position.y

        q = [0]*4
        q[0] = wp.pose.orientation.x
        q[1] = wp.pose.orientation.y
        q[2] = wp.pose.orientation.z
        q[3] = wp.pose.orientation.w

        R = kdl.Rotation.Quaternion(q[0],q[1],q[2],q[3])
        rpy = R.GetRPY()
        p.theta = rpy[2] 

        n = geometry_msgs.msg.Vector3(R[0,2],R[1,2],R[2,2])
        
        goal.path.append(p)
        goal.heights.append(wp.pose.position.z)
        goal.normals.append(n)
    
        
        goal.step_duration = rospy.get_param("~duration")
        goal.step_mode = True
        goal.relative_height = False
        goal.mode = 4

        rospy.loginfo("AtlasPathPlanner::execute_leg_plan() -- sending walk path goal")
        # Sends the goal to the action server.
        self.walk_controller_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        rospy.loginfo("AtlasPathPlanner::execute_leg_plan() -- waiting for walk path result")
        self.walk_controller_client.wait_for_result()

        # Prints out the result of executing the action
        # print self.walk_controller_client.get_state()
        return self.walk_controller_client.get_state() == GoalStatus.SUCCEEDED


    ##################################
    ######## PLANNING METHODS ########
    ##################################
  
    def plan_to_cartesian_goal(self, group_name, pt) :
    
        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal()")

        if "_leg" in group_name :
            if self.auto_execute[group_name] :
                rospy.loginfo(str("AtlasPathPlanner::plan_to_cartesian_goal() -- using execute_leg_plan() for " + group_name))
                self.execute_leg_plan(group_name, pt)
            else :
                rospy.logwarn(str("AtlasPathPlanner::plan_to_cartesian_goal() -- no route to just plan for leg: " + group_name))
            return None

        print "goal:"
        print pt
        print "-----"
        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal() -- waiting for server")
        self.cartesian_reach_client.wait_for_server()

        goal = matec_actions.msg.PlannedManipulationGoal()

        goal.allow_incomplete_planning = rospy.get_param("~allow_incomplete_planning")
        goal.num_acceptable_consecutive_failures = rospy.get_param("~num_acceptable_consecutive_failures")
        goal.plan_visualization_density = rospy.get_param("~plan_visualization_density")
        goal.visualize_on_plan = rospy.get_param("~visualize_on_plan")
        goal.execute_on_plan = self.auto_execute[group_name]

        goal.plan_name = group_name
        self.last_plan_name = goal.plan_name
        rospy.logwarn("Setting plan_name: " + self.last_plan_name)

        motion = matec_msgs.msg.GoalMotion()

        try :
            for idx in range(len(self.get_group_joints(group_name))) :
                if self.groups[group_name].joint_mask.mask[idx] :
                    motion.available_joints.append(self.get_group_joints(group_name)[idx])  
        except :          
            motion.available_joints = self.get_group_joints(group_name)
    
        motion.max_angular_velocity = rospy.get_param("~max_angular_velocity")
        motion.max_linear_velocity = rospy.get_param("~max_linear_velocity")
        motion.stable_frame = self.srdf_model.get_base_link(group_name)
        motion.segment_duration = 0.0

        final_region = matec_msgs.msg.GoalRegion()
        initial_region = matec_msgs.msg.GoalRegion()

        final_region.tool_frame = self.srdf_model.get_tip_link(group_name)
        pos_tol = self.get_goal_position_tolerances(group_name)
        rot_tol = self.get_goal_orientation_tolerances(group_name)
        final_region.x.min = -pos_tol[0]
        final_region.x.max = pos_tol[0]
        final_region.y.min = -pos_tol[1]
        final_region.y.max = pos_tol[1]
        final_region.z.min = -pos_tol[2]
        final_region.z.max = pos_tol[2]
        final_region.R.min = -rot_tol[0]
        final_region.R.max = rot_tol[0]
        final_region.P.min = -rot_tol[1]
        final_region.P.max = rot_tol[1]
        final_region.Y.min = -rot_tol[2]
        final_region.Y.max = rot_tol[2]
        initial_region = copy.deepcopy(final_region)

        final_region.goal_frame = pt
        final_region.goal_frame.header.stamp = rospy.Time(0)
        final_region.goal_frame.header.frame_id = final_region.goal_frame.header.frame_id.lstrip("/")

        motion.final_goal_regions.append(final_region)

        initial_region.goal_frame.header.frame_id = self.srdf_model.get_tip_link(group_name).lstrip("/")
        initial_region.goal_frame.header.stamp = rospy.Time(0)
        initial_region.goal_frame.pose.orientation.w = 1.0
        motion.initial_goal_regions.append(initial_region)

        goal.segments.append(motion);

        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)

        # rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal() -- waiting for result")
        # finished_before_timeout = self.cartesian_reach_client.wait_for_result(rospy.Duration(20))
    
        return None

        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal() -- got result")
        resp = self.cartesian_reach_client.get_feedback()
        
        return resp.visualization_plan.trajectory


    def plan_to_joint_goal(self, group_name, js) :

        rospy.loginfo("AtlasPathPlanner::plan_to_joint_goal()")

        if self.group_types[group_name] == "endeffector" :
            plan = trajectory_msgs.msg.JointTrajectory()
            point = trajectory_msgs.msg.JointTrajectoryPoint()

            plan.header.stamp = rospy.Time.now()
            for j in range(len(js.name)) :
                plan.joint_names.append(js.name[j])
                point.positions.append(js.position[j])
            plan.points.append(point)
            return plan


        # resp = JointPlanCommandResponse()

        rospy.loginfo("AtlasPathPlanner::plan_to_joint_goal() -- waiting for server")
        self.joint_action_client.wait_for_server()
        
        # Creates a goal to send to the action server.
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = trajectory_msgs.msg.JointTrajectory()
        goal.trajectory.joint_names = js.name
        jp = trajectory_msgs.msg.JointTrajectoryPoint()
        jp.positions = js.position
        goal.trajectory.points.append(jp)

        rospy.loginfo("AtlasPathPlanner::plan_to_joint_goal() -- sending goal")
        
        # Sends the goal to the action server.
        self.joint_action_client.send_goal(goal)

        # # Waits for the server to finish performing the action.
        # rospy.loginfo("AtlasPathPlanner::plan_to_joint_goal() -- waiting for result")
        # self.joint_action_client.wait_for_result()

        # return goal.trajectory
        return trajectory_msgs.msg.JointTrajectory()

    def plan_to_random_goal(self, group_name) :
        rospy.logwarn("AtlasPathPlanner::plan_to_random_goal() -- NOT IMPLEMENETED YET")
        return None

  
    def plan_cartesian_path(self, group_name, waypoints) :

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path()")

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- waiting for server")
        self.cartesian_reach_client.wait_for_server()
        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- server found, constructing goal")
        
        goal = matec_actions.msg.PlannedManipulationGoal()

        goal.allow_incomplete_planning = rospy.get_param("~allow_incomplete_planning")
        goal.num_acceptable_consecutive_failures = rospy.get_param("~num_acceptable_consecutive_failures")
        goal.plan_visualization_density = rospy.get_param("~plan_visualization_density")
        goal.execute_on_plan = self.auto_execute[group_name]

        goal.plan_name = group_name
        self.last_plan_name = goal.plan_name
        rospy.logwarn("Setting plan_name: " + self.last_plan_name)
        rospy.logwarn("Setting plan frame: " + self.get_group_planning_frame(group_name))

        for wp in waypoints :

            motion = matec_msgs.msg.GoalMotion()
            motion.available_joints = self.get_group_joints(group_name)
            motion.max_angular_velocity = rospy.get_param("~max_angular_velocity")
            motion.max_linear_velocity = rospy.get_param("~max_linear_velocity")
            motion.stable_frame = self.srdf_model.get_base_link(group_name)
            motion.segment_duration = 0.0

            final_region = matec_msgs.msg.GoalRegion()
            initial_region = matec_msgs.msg.GoalRegion()

            final_region.tool_frame = self.srdf_model.get_tip_link(group_name)
            pos_tol = self.get_goal_position_tolerances(group_name)
            rot_tol = self.get_goal_orientation_tolerances(group_name)
            final_region.x.min = -pos_tol[0]
            final_region.x.max = pos_tol[0]
            final_region.y.min = -pos_tol[1]
            final_region.y.max = pos_tol[1]
            final_region.z.min = -pos_tol[2]
            final_region.z.max = pos_tol[2]
            final_region.R.min = -rot_tol[0]
            final_region.R.max = rot_tol[0]
            final_region.P.min = -rot_tol[1]
            final_region.P.max = rot_tol[1]
            final_region.Y.min = -rot_tol[2]
            final_region.Y.max = rot_tol[2]
            initial_region = copy.deepcopy(final_region)

            p = geometry_msgs.msg.PoseStamped()
            p.header.stamp = rospy.Time(0)
            p.header.frame_id = self.get_group_planning_frame(group_name).lstrip("/")
            p.pose = wp

            final_region.goal_frame = p
            motion.final_goal_regions.append(final_region)
            initial_region.goal_frame.header.frame_id = self.srdf_model.get_tip_link(group_name).lstrip("/")

            initial_region.goal_frame.header.stamp = rospy.Time(0)
            initial_region.goal_frame.pose.orientation.w = 1.0
            motion.initial_goal_regions.append(initial_region)

        goal.segments.append(motion);

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)


        # rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- waiting for result")
        # self.cartesian_reach_client.wait_for_result()
        # rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- got result")
        # resp = self.cartesian_reach_client.get_result()
        # print resp

        while self.cartesian_reach_client.get_state()==0 :
            print self.cartesian_reach_client.get_state
            rospy.sleep(0.01)

        print "Getting plan from service"
        p = self.get_plan()
        print p
        return p
        
    def plan_navigation_path(self, waypoints) :

        rospy.loginfo("AtlasPathPlanner::plan_navigation_path()")
        
        req = PlanStepsRequest()

        for wp in waypoints :
            req.target.append(wp)
            
        req.plan_through_unknown_cells = True
        req.solver_timeout = 10.0

        try :
            rospy.wait_for_service("/plan_steps", self.wait_for_service_timeout)
        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::plan_navigation_path(): " + str(e))
            return None

        try :
            rospy.loginfo("AtlasPathPlanner::plan_navigation_path() -- requesting plan!")
            step_planner = rospy.ServiceProxy("/plan_steps", PlanSteps)
            resp = step_planner(req)
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::plan_navigation_path() -- " + str(e)))
            return None

        rospy.loginfo("AtlasPathPlanner::plan_navigation_path() -- got footsteps!")

        if resp.left_foot_start :            
            rospy.set_param("~start_foot", "left")
        else :
            rospy.set_param("~start_foot", "right")
        
        return resp.steps


    ### multigroup functions
    def plan_to_cartesian_goals(self, group_names, pts) :

        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goals()")

        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goals() -- waiting for server")
        self.cartesian_reach_client.wait_for_server()

        goal = matec_actions.msg.PlannedManipulationGoal()

        goal.allow_incomplete_planning = rospy.get_param("~allow_incomplete_planning")
        goal.num_acceptable_consecutive_failures = rospy.get_param("~num_acceptable_consecutive_failures")
        goal.plan_visualization_density = rospy.get_param("~plan_visualization_density")
        goal.execute_on_plan = True in [self.auto_execute[g] for g in group_names]

        goal.plan_name = ""
        for g in group_names :
            goal.plan_name += str("/") + g

        self.last_plan_name = goal.plan_name
        rospy.logwarn("Setting plan_name: " + self.last_plan_name)

        for idx in range(len(group_names)) :
            group_name = group_names[idx]
            pt = pts[idx]

            motion = matec_msgs.msg.GoalMotion()
            motion.available_joints = self.get_group_joints(group_name)
            motion.max_angular_velocity = rospy.get_param("~max_angular_velocity")
            motion.max_linear_velocity = rospy.get_param("~max_linear_velocity")
            motion.stable_frame = self.srdf_model.get_base_link(group_name)
            motion.segment_duration = 0.0

            final_region = matec_msgs.msg.GoalRegion()
            initial_region = matec_msgs.msg.GoalRegion()

            final_region.tool_frame = self.srdf_model.get_tip_link(group_name)
            pos_tol = self.get_goal_position_tolerances(group_name)
            rot_tol = self.get_goal_orientation_tolerances(group_name)
            final_region.x.min = -pos_tol[0]
            final_region.x.max = pos_tol[0]
            final_region.y.min = -pos_tol[1]
            final_region.y.max = pos_tol[1]
            final_region.z.min = -pos_tol[2]
            final_region.z.max = pos_tol[2]
            final_region.R.min = -rot_tol[0]
            final_region.R.max = rot_tol[0]
            final_region.P.min = -rot_tol[1]
            final_region.P.max = rot_tol[1]
            final_region.Y.min = -rot_tol[2]
            final_region.Y.max = rot_tol[2]
            initial_region = copy.deepcopy(final_region)

            final_region.goal_frame = pt
            final_region.goal_frame.header.stamp = rospy.Time(0)
            final_region.goal_frame.header.frame_id = final_region.goal_frame.header.frame_id.lstrip("/")

            motion.final_goal_regions.append(final_region)

            initial_region.goal_frame.header.frame_id = self.srdf_model.get_tip_link(group_name).lstrip("/")
            initial_region.goal_frame.header.stamp = rospy.Time(0)
            initial_region.goal_frame.pose.orientation.w = 1.0
            motion.initial_goal_regions.append(initial_region)

            goal.segments.append(motion);


        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goals() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)

        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goals() -- waiting for result")
        finished_before_timeout = self.cartesian_reach_client.wait_for_result(rospy.Duration(20))

        if finished_before_timeout :
            rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goals() -- got result")
            resp = self.cartesian_reach_client.get_status()
            print resp

            print resp.visualization_plan.trajectory

            return resp.visualization_plan.trajectory
        else :
            rospy.logwarn("AtlasPathPlanner::plan_to_cartesian_goals() -- cartesian reach client timed out")
            return visualization_msgs.msg.JointTrajectory()

        # NEED TO RETURN ARRAY OR TRAJs


    def plan_to_joint_goals(self, group_names, jss) :
        r = []
        if len(group_names) == len(jss) :
            js = sensor_msgs.msg.JointState()
            for idx in range(len(group_names)) :
                js.name.append(jss[idx].name)
                js.position.append(jss[idx].position)

            r.append(self.plan_to_joint_goal("combined_joint_group", js))
        else :
            rospy.logerr("AtlasPathPlanner::plan_to_joint_goals() -- input arg size mismatch")
        return r


    def plan_to_random_goals(self, group_names) :
        r = []
        for i in len(group_names) :
            r.append(self.plan_to_random_goal(group_names[i]))
        return r


    def plan_cartesian_paths(self, group_names, pt_lists) :
        
        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths()")

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- waiting for server")
        self.cartesian_reach_client.wait_for_server()

        goal = matec_actions.msg.PlannedManipulationGoal()

        goal.visualize_on_plan = rospy.get_param("~visualize_on_plan")
        goal.allow_incomplete_planning = rospy.get_param("~allow_incomplete_planning")
        goal.num_acceptable_consecutive_failures = rospy.get_param("~num_acceptable_consecutive_failures")
        goal.plan_visualization_density = rospy.get_param("~plan_visualization_density")
        goal.execute_on_plan = True in [self.auto_execute[g] for g in group_names]
       
        goal.plan_name = ""
        for g in group_names :
            goal.plan_name += str("/") + g

        self.last_plan_name = goal.plan_name
        rospy.logwarn("Setting plan_name: " + self.last_plan_name)

        num_groups = len(group_names)
        num_segments = len(pt_lists[0])

        segments = []
        for idx in range(num_segments) :
            segment = []
            for jdx in range(num_groups) :
                segment.append(pt_lists[jdx][idx])
            segments.append(segment)

        for segment in segments :

            motion = matec_msgs.msg.GoalMotion()
            motion.max_angular_velocity = rospy.get_param("~max_angular_velocity")
            motion.max_linear_velocity = rospy.get_param("~max_linear_velocity")
            motion.stable_frame = self.get_robot_planning_frame()
            motion.segment_duration = 0.0
            motion.available_joints = []

            idx = 0
            for wp in segment :

                group_name = group_names[idx]
                joints = list(set(motion.available_joints + self.get_group_joints(group_name)))
                motion.available_joints = joints

                final_region = matec_msgs.msg.GoalRegion()

                final_region.tool_frame = self.srdf_model.get_tip_link(group_name)
                pos_tol = self.get_goal_position_tolerances(group_name)
                rot_tol = self.get_goal_orientation_tolerances(group_name)
                final_region.x.min = -pos_tol[0]
                final_region.x.max = pos_tol[0]
                final_region.y.min = -pos_tol[1]
                final_region.y.max = pos_tol[1]
                final_region.z.min = -pos_tol[2]
                final_region.z.max = pos_tol[2]
                final_region.R.min = -rot_tol[0]
                final_region.R.max = rot_tol[0]
                final_region.P.min = -rot_tol[1]
                final_region.P.max = rot_tol[1]
                final_region.Y.min = -rot_tol[2]
                final_region.Y.max = rot_tol[2]

                # final_region.x.min = -0.01
                # final_region.x.max = 0.01
                # final_region.y.min = -0.01
                # final_region.y.max = 0.01
                # final_region.z.min = -0.01
                # final_region.z.max = 0.01
                # final_region.R.min = -3.14
                # final_region.R.max = 3.14
                # final_region.P.min = -3.14
                # final_region.P.max = 3.14
                # final_region.Y.min = -3.14
                # final_region.Y.max = 3.14

                final_region.goal_frame.pose = wp.pose
                final_region.goal_frame.header.stamp = rospy.Time(0)
                final_region.goal_frame.header.frame_id = wp.header.frame_id.lstrip("/")
                motion.final_goal_regions.append(final_region)

                if len(goal.segments) > 0 :                   
                    n = len(goal.segments) - 1
                    prev_final_region = goal.segments[n].final_goal_regions[idx]
                    initial_region = copy.deepcopy(prev_final_region)
                    
                else :
                    initial_region = matec_msgs.msg.GoalRegion()
                    initial_region = copy.deepcopy(final_region)
                    initial_region.goal_frame.header.frame_id = self.srdf_model.get_tip_link(group_name).lstrip("/")
                    initial_region.goal_frame.header.stamp = rospy.Time(0)
                    initial_region.goal_frame.pose = geometry_msgs.msg.Pose()
                    initial_region.goal_frame.pose.orientation.w = 1.0

                motion.initial_goal_regions.append(initial_region)

                idx += 1

            goal.segments.append(motion);

        # print goal

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- polling feedback")
        fb_msg = rospy.wait_for_message("/planned_manipulation/server/feedback", matec_actions.msg.PlannedManipulationActionFeedback)

        while not fb_msg.feedback.planning_complete :
            fb_msg = rospy.wait_for_message("/planned_manipulation/server/feedback", matec_actions.msg.PlannedManipulationActionFeedback)
            
        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- PLANNING COMPLETE")
        if fb_msg.feedback.planning_progress > 0.0 :            
            p = self.get_plan()
            return p
        else :
            return None


    def clear_goal_targets(self, group_names) :
        for g in group_names :
            self.clear_goal_target(g)

    def has_joint_map(self, group_name) :
        return True

    def get_joint_map(self, group_name) :
        return self.groups[group_name].joint_map

    def get_joint_mask(self, group_name) :
        return self.groups[group_name].joint_mask.mask

    def set_joint_mask(self, group_name, mask) :
        self.groups[group_name].joint_mask.mask = mask
        self.srdf_model.set_joint_mask(group_name, mask)

    def planner_feedback(self, data) :
        if data.feedback.planning_complete :

            if data.feedback.planning_progress > 0.9 :
                self.plan_generated[self.last_plan_name] = True
            print "planner complete for ", self.last_plan_name
            jt = self.get_plan() 
            self.process_plan_viz(jt)

    def planner_viz_feedback(self, data) :
        jt = data.trajectory
        self.process_plan_viz(jt)

    def get_plan(self) :

        rospy.wait_for_service('/planned_manipulation/visualize')
        # self.connected_to_plan_viz = rospy.wait_for_service('/planned_manipulation/visualize', 3.0)
        # if not self.connected_to_plan_viz :
        #     rospy.loginfo("AtlasPathPlanner::get_plan_viz() -- waiting for visualization service")  
        #     self.connected_to_plan_viz = rospy.wait_for_service('/planned_manipulation/visualize', 3.0)
        #     rospy.logwarn("AtlasPathPlanner::get_plan_viz() -- timeout out waiting for visualization service")  
        #     if not self.connected_to_plan_viz :
        #         rospy.logerr("AtlasPathPlanner::get_plan_viz() -- failed waiting for visualization service")  
        #         return

        try:
            req = VisualizeManipulationPlanRequest()
            req.plan_names.append(self.last_plan_name)
            req.plans_in_parallel = False
            req.plan_visualization_density = rospy.get_param("~plan_visualization_density")
            get_plan = rospy.ServiceProxy('/planned_manipulation/visualize', VisualizeManipulationPlan)
            resp = get_plan(req)
        except rospy.ServiceException, e:
            print "AtlasPathPlanner::get_plan_viz() -- service call failed: %s"%e
            return None

        rospy.loginfo("AtlasPathPlanner::get_plan_viz() -- publishing goal regions")
        self.goal_region_pub.publish(resp.goal_regions)     

        return resp.joint_trajectories.trajectory

    def process_plan_viz(self, jt) :
        
        joint_names = jt.joint_names
        group = self.get_group_from_names(joint_names)
        if group != "" :
            rospy.loginfo(str("AtlasPathPlanner::get_plan_viz() -- trajectory for " + group))
     
            jm = self.groups[group].joint_map           
            jt_ordered = trajectory_msgs.msg.JointTrajectory()
            new_map = {}

            # print "group joints: ", self.get_group_joints(group) 
            # print "traj joints:  ", joint_names
            for j in self.get_group_joints(group) :
                try :
                    if j in joint_names :
                        idx = joint_names.index(j)
                        new_map[j] = idx
                        jt_ordered.joint_names.append(j)
                except :
                    pass

            for p in jt.points :
                if len(p.positions) != len(jt_ordered.joint_names) :
                    continue
                # print p.positions
                ordered_p = trajectory_msgs.msg.JointTrajectoryPoint()
                for j in jt_ordered.joint_names :
                    # print j
                    # print new_map[j]
                    # print p.positions[new_map[j]]
                    ordered_p.positions.append(p.positions[new_map[j]])
                jt_ordered.points.append(ordered_p)

            self.publish_path_data(jt_ordered, group)

    def get_group_from_names(self, joint_names) :
        if len(joint_names) > 0 :
            for g in self.groups.keys() :
                joints = self.get_group_joints(g)
                found = True
                for j in joint_names :
                    if j not in joints: 
                        found = False
                if found: 
                    return g
        rospy.logwarn("AtlasPathPlanner::get_group_from_names() -- couldn't find group from feedback joints")
        return ""

if __name__=="__main__":

    rospy.init_node("atlas_planner_client")
    pp = AtlasPathPlanner("atlas", "atlas_moveit_config")
    rospy.spin()
