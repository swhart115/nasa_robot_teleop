#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import PyKDL as kdl

roslib.load_manifest('step_finder')
roslib.load_manifest('walk_controller')

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg

import actionlib
from actionlib_msgs.msg import GoalStatus
from matec_msgs.msg import *
from matec_actions.msg import *
from walk_controller.msg import *

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
        self.planning_frame = rospy.get_param("/interactive_control/navigation_frame", "/global")
        self.groups = {}
        self.joint_names = []
        self.feet_names = ['left', 'right']
        self.wait_for_service_timeout = 5.0

        rospy.set_param("~interpolation_type", 1)
        rospy.set_param("~duration", 2.0)
        rospy.set_param("~num_visualizaton_points", 5)
        rospy.set_param("~visualize_path", True)
        rospy.set_param("~maintain_hand_pose_offsets", False)
        rospy.set_param("~move_as_far_as_possible", False)

        rospy.set_param("~allow_incomplete_planning", True)
        rospy.set_param("~num_acceptable_consecutive_failures", 0)
        rospy.set_param("~plan_visualization_density", 0.5)
        
        self.cartesian_reach_client = actionlib.SimpleActionClient('/planned_manipulation/server', matec_actions.msg.PlannedManipulationAction)
        self.joint_action_client = actionlib.SimpleActionClient('/base_joint_interpolator/server', control_msgs.msg.FollowJointTrajectoryAction)
        self.walk_controller_client = actionlib.SimpleActionClient('/path_walker', walk_controller.msg.WalkPathAction)
   
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
        rospy.loginfo(str("AtlasPathPlanner::execute_plan(" + group_name+ ")"))

        # hack to send individual foot goal to robot
        if "_leg" in group_name :
            rospy.logwarn("AtlasPathPlanner::execute_plan() -- can't execute leg command directly; use \'execute_on_plan\' instead")
            return False

        req = ExecuteCommandRequest()
        req.groups.append(group_name)

        try :
            rospy.wait_for_service("/interactive_controls_bridge/execute_command", self.wait_for_service_timeout)
        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::execute_plan(): " + str(e))
            return False

        try :
            executor = rospy.ServiceProxy("/interactive_controls_bridge/execute_command", ExecuteCommand)
            resp = executor(req)
            for p in resp.progress :
                rospy.loginfo(str("AtlasPathPlanner::execute_plan(" + group_name + ") progress: " + str(p)))
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

    # def plan_to_cartesian_goal(self, group_name, pt) :
        
    #     rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal()")

    #     req = CartesianPlanCommandRequest()
        
    #     interpolation_type = rospy.get_param("~interpolation_type")
    #     duration = rospy.get_param("~duration")
    #     num_visualizaton_points = rospy.get_param("~num_visualizaton_points")

    #     req.return_trajectories = rospy.get_param("~visualize_path")
    #     req.maintain_hand_pose_offsets = rospy.get_param("~maintain_hand_pose_offsets")
    #     req.move_as_far_as_possible = rospy.get_param("~move_as_far_as_possible")

    #     if group_name in self.groups.keys() :

    #         req.execute_on_plan = self.auto_execute[group_name]

    #         got = self.get_goal_orientation_tolerances(group_name)
    #         gpt = self.get_goal_position_tolerances(group_name)

    #         gotv = geometry_msgs.msg.Vector3(got[0],got[1],got[2])
    #         gptv = geometry_msgs.msg.Vector3(gpt[0],gpt[1],gpt[2])

    #         spec = CartesianPlanRequestSpecification()
    #         spec.group_id = self.groups[group_name].group_id
    #         spec.group_name = self.groups[group_name].group_name
    #         spec.joint_mask = self.groups[group_name].joint_mask
    #         spec.control_frame = self.groups[group_name].control_frame
    #         spec.waypoints.append(pt)
    #         spec.duration.append(duration)
    #         spec.num_visualizaton_points = num_visualizaton_points
    #         spec.angle_variance.append(gotv)
    #         spec.maximum_angle_variance.append(gotv)
    #         spec.position_variance.append(gptv)
    #         spec.maximum_position_variance.append(gptv)
    #         spec.interpolation_type = interpolation_type
    #         req.group_plan_specs.append(spec)

    #     else :
    #         rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- no group found of that name!"))
    #         return None

    #     try :
    #         rospy.wait_for_service("/interactive_controls_bridge/cartesian_plan_command", self.wait_for_service_timeout)
    #     except rospy.ROSException as e:
    #         rospy.logerr("AtlasPathPlanner::plan_to_cartesian_goal(): " + str(e))
    #         return None

    #     try :
    #         planner = rospy.ServiceProxy("/interactive_controls_bridge/cartesian_plan_command", CartesianPlanCommand)
    #         resp = planner(req)
    #         if len(resp.result) > 0 :
    #             return resp.result[0]
    #         else :
    #             rospy.logwarn(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- failed to get plan to goal"))
    #             return None
    #     except rospy.ServiceException, e:
    #         rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- CartesianPlanCommand service call failed: " + str(e)))
    #         return None


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
        goal.execute_on_plan = self.auto_execute[group_name]

        motion = matec_msgs.msg.GoalMotion()
        motion.available_joints = self.get_group_joints(group_name)
        motion.max_angular_velocity = 0.2
        motion.max_linear_velocity = 0.2
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

        print goal
        print "------\n DONE"

        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)

        # rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal() -- waiting for result")
        # finished_before_timeout = self.cartesian_reach_client.wait_for_result(rospy.Duration(20))

        return None
        
        rospy.loginfo("AtlasPathPlanner::plan_to_cartesian_goal() -- got result")
        resp = self.cartesian_reach_client.get_feedback()
        
        print resp

        print resp.visualization_plan.trajectory

        return resp.visualization_plan.trajectory


        # else :
        #     rospy.logwarn("AtlasPathPlanner::plan_to_cartesian_goal() -- cartesian reach client timed out")
        #     return visualization_msgs.msg.JointTrajectory()

    # def plan_to_joint_goal(self, group_name, js) :

    #     rospy.loginfo("AtlasPathPlanner::plan_to_joint_goal()")

    #     if self.group_types[group_name] == "endeffector" :
    #         plan = trajectory_msgs.msg.JointTrajectory()
    #         point = trajectory_msgs.msg.JointTrajectoryPoint()

    #         plan.header.stamp = rospy.Time.now()
    #         for j in range(len(js.name)) :
    #             plan.joint_names.append(js.name[j])
    #             point.positions.append(js.position[j])
    #         plan.points.append(point)
    #         return plan

    #     req = JointPlanCommandRequest()

    #     num_visualizaton_points = rospy.get_param("~num_visualizaton_points")
    #     duration = rospy.get_param("~duration")

    #     req.return_trajectories = rospy.get_param("~visualize_path")

    #     if group_name in self.groups.keys() :
    #         req.execute_on_plan = self.auto_execute[group_name]

    #         spec = JointPlanRequestSpecification()
    #         spec.num_visualizaton_points = num_visualizaton_points

    #         goal = control_msgs.msg.FollowJointTrajectoryGoal()
    #         goal.trajectory = trajectory_msgs.msg.JointTrajectory()
    #         goal.trajectory.header = js.header
    #         goal.trajectory.joint_names = js.name

    #         point = trajectory_msgs.msg.JointTrajectoryPoint()
    #         point.positions = js.position
    #         point.velocities = js.velocity
    #         point.time_from_start = rospy.Duration(duration)
    #         goal.trajectory.points.append(point)

    #         for n in js.name :
    #             tol = control_msgs.msg.JointTolerance()
    #             tol.name = n
    #             tol.position = self.joint_tolerance[group_name]
    #             goal.path_tolerance.append(tol)
    #             goal.goal_tolerance.append(tol)

    #         spec.waypoints.append(goal)
    #         req.group_plan_specs.append(spec)

    #     try :
    #         rospy.wait_for_service("/interactive_controls_bridge/joint_plan_command", self.wait_for_service_timeout)
    #     except rospy.ROSException as e:
    #         rospy.logerr("AtlasPathPlanner::plan_to_joint_goal(): " + str(e))
    #         return None

    #     try :
    #         planner = rospy.ServiceProxy("/interactive_controls_bridge/joint_plan_command", JointPlanCommand)
    #         resp = planner(req)
    #         return resp.result[0]
    #     except rospy.ServiceException, e:
    #         rospy.logerr(str("AtlasPathPlanner::plan_to_joint_goal(" + group_name + ") -- JointPlanCommand service call failed: " + str(e)))
    #         return None

    def plan_to_joint_goal(self, group_name, js) :

        rospy.loginfo("AtlasPathPlanner::plan_to_joint_goal()")

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

        # try :
        #     self.groups[group_name].set_random_target() # FIXME
        #     plan = self.groups[group_name].plan()  # FIXME
        #     return plan.joint_trajectory
        # except :
        #     rospy.logwarn(str("AtlasPathPlanner::plan_to_random_goal(" + group_name + ") -- failed"))
        #     return None

    # def plan_cartesian_path(self, group_name, waypoints) :

    #     rospy.loginfo("AtlasPathPlanner::plan_cartesian_path()")
    #     req = CartesianPlanCommandRequest()

    #     interpolation_type = rospy.get_param("~interpolation_type")
    #     duration = rospy.get_param("~duration")
    #     num_visualizaton_points = rospy.get_param("~num_visualizaton_points")

    #     req.return_trajectories = rospy.get_param("~visualize_path")
    #     req.maintain_hand_pose_offsets = rospy.get_param("~maintain_hand_pose_offsets")
    #     req.move_as_far_as_possible = rospy.get_param("~move_as_far_as_possible")

    #     if group_name in self.groups.keys() :

    #         req.execute_on_plan = self.auto_execute[group_name]

    #         spec = CartesianPlanRequestSpecification()
    #         spec.group_id = self.groups[group_name].group_id
    #         spec.group_name = self.groups[group_name].group_name
    #         spec.joint_mask = self.groups[group_name].joint_mask
    #         spec.control_frame = self.groups[group_name].control_frame

    #         for wp in waypoints :

    #             ps = geometry_msgs.msg.PoseStamped() 
    #             ps.pose = wp
    #             ps.header.stamp = rospy.Time.now()
    #             ps.header.frame_id = self.get_group_planning_frame(group_name)
    #             got = self.get_goal_orientation_tolerances(group_name)
    #             gpt = self.get_goal_position_tolerances(group_name)
    #             gotv = geometry_msgs.msg.Vector3(got[0],got[1],got[2])
    #             gptv = geometry_msgs.msg.Vector3(gpt[0],gpt[1],gpt[2])

    #             spec.waypoints.append(ps)
    #             spec.duration.append(duration)
    #             spec.angle_variance.append(gotv)
    #             spec.maximum_angle_variance.append(gotv)
    #             spec.position_variance.append(gptv)
    #             spec.maximum_position_variance.append(gptv)

    #         spec.interpolation_type = interpolation_type
    #         spec.num_visualizaton_points = num_visualizaton_points
    #         req.group_plan_specs.append(spec)

    #     else :
    #         rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- no group found of that name!"))
    #         return None

    #     try :
    #         rospy.wait_for_service("/interactive_controls_bridge/cartesian_plan_command", self.wait_for_service_timeout)
    #     except rospy.ROSException as e:
    #         rospy.logerr("AtlasPathPlanner::plan_to_cartesian_goal(): " + str(e))
    #         return None

    #     try :
    #         planner = rospy.ServiceProxy("/interactive_controls_bridge/cartesian_plan_command", CartesianPlanCommand)
    #         resp = planner(req)
    #         return resp.result[0]
    #     except rospy.ServiceException, e:
    #         rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- PlanCommand service call failed: " + str(e)))
    #         return None

    def plan_cartesian_path(self, group_name, waypoints) :

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path()")

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- waiting for server")
        self.cartesian_reach_client.wait_for_server()

        goal = matec_actions.msg.PlannedManipulationGoal()

        goal.allow_incomplete_planning = rospy.get_param("~allow_incomplete_planning")
        goal.num_acceptable_consecutive_failures = rospy.get_param("~num_acceptable_consecutive_failures")
        goal.plan_visualization_density = rospy.get_param("~plan_visualization_density")
        goal.execute_on_plan = self.auto_execute[group_name]

        for wp in waypoints :
            motion = matec_msgs.msg.GoalMotion
            motion.available_joints = self.get_group_joints(group_name)
            motion.max_angular_velocity = 0.2
            motion.max_linear_velocity = 0.2
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
            initial_region = final_region

            final_region.goal_frame = wp
            final_region.goal_frame.header.stamp = rospy.Time(0)
            final_region.goal_frame.header.frame_id = final_region.goal_frame.header.frame_id.lstrip("/")
            motion.final_goal_regions.append(final_region)

            initial_region.goal_frame.header.frame_id = self.srdf_model.get_tip_link(group_name).lstrip("/")

            initial_region.goal_frame.header.stamp = rospy.Time(0)
            initial_region.goal_frame.pose.orientation.w = 1.0
            motion.initial_goal_regions.append(initial_region)

        goal.segments.append(motion);

        print goal

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- waiting for result")
        self.cartesian_reach_client.wait_for_result()

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_path() -- got result")
        resp = self.cartesian_reach_client.get_result()

        print resp.visualization_plan.trajectory

        return resp.visualization_plan.trajectory

        # interpolation_type = rospy.get_param("~interpolation_type")
        # duration = rospy.get_param("~duration")
        # num_visualizaton_points = rospy.get_param("~num_visualizaton_points")

        # req.return_trajectories = rospy.get_param("~visualize_path")
        # req.maintain_hand_pose_offsets = rospy.get_param("~maintain_hand_pose_offsets")
        # req.move_as_far_as_possible = rospy.get_param("~move_as_far_as_possible")

        # if group_name in self.groups.keys() :

        #     req.execute_on_plan = self.auto_execute[group_name]

        #     spec = CartesianPlanRequestSpecification()
        #     spec.group_id = self.groups[group_name].group_id
        #     spec.group_name = self.groups[group_name].group_name
        #     spec.joint_mask = self.groups[group_name].joint_mask
        #     spec.control_frame = self.groups[group_name].control_frame

        #     for wp in waypoints :

        #         ps = geometry_msgs.msg.PoseStamped() 
        #         ps.pose = wp
        #         ps.header.stamp = rospy.Time.now()
        #         ps.header.frame_id = self.get_group_planning_frame(group_name)
        #         got = self.get_goal_orientation_tolerances(group_name)
        #         gpt = self.get_goal_position_tolerances(group_name)
        #         gotv = geometry_msgs.msg.Vector3(got[0],got[1],got[2])
        #         gptv = geometry_msgs.msg.Vector3(gpt[0],gpt[1],gpt[2])

        #         spec.waypoints.append(ps)
        #         spec.duration.append(duration)
        #         spec.angle_variance.append(gotv)
        #         spec.maximum_angle_variance.append(gotv)
        #         spec.position_variance.append(gptv)
        #         spec.maximum_position_variance.append(gptv)

        #     spec.interpolation_type = interpolation_type
        #     spec.num_visualizaton_points = num_visualizaton_points
        #     req.group_plan_specs.append(spec)

        # else :
        #     rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- no group found of that name!"))
        #     return None

        # try :
        #     rospy.wait_for_service("/interactive_controls_bridge/cartesian_plan_command", self.wait_for_service_timeout)
        # except rospy.ROSException as e:
        #     rospy.logerr("AtlasPathPlanner::plan_to_cartesian_goal(): " + str(e))
        #     return None

        # try :
        #     planner = rospy.ServiceProxy("/interactive_controls_bridge/cartesian_plan_command", CartesianPlanCommand)
        #     resp = planner(req)
        #     return resp.result[0]
        # except rospy.ServiceException, e:
        #     rospy.logerr(str("AtlasPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- PlanCommand service call failed: " + str(e)))
        #     return None

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

        for idx in range(len(group_names)) :
            group_name = group_names[idx]
            pt = pts[idx]

            motion = matec_msgs.msg.GoalMotion()
            motion.available_joints = self.get_group_joints(group_name)
            motion.max_angular_velocity = 0.2
            motion.max_linear_velocity = 0.2
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

        print goal
        print "------\n DONE"

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



    def plan_cartesian_paths(self, group_names, frame_ids, pt_lists) :
        
        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths()")

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- waiting for server")
        self.cartesian_reach_client.wait_for_server()

        goal = matec_actions.msg.PlannedManipulationGoal()

        goal.allow_incomplete_planning = rospy.get_param("~allow_incomplete_planning")
        goal.num_acceptable_consecutive_failures = rospy.get_param("~num_acceptable_consecutive_failures")
        goal.plan_visualization_density = rospy.get_param("~plan_visualization_density")
        goal.execute_on_plan = True in [self.auto_execute[g] for g in group_names]
       

        for wp in waypoints :
            motion = matec_msgs.msg.GoalMotion
            motion.available_joints = self.get_group_joints(group_name)
            motion.max_angular_velocity = 0.2
            motion.max_linear_velocity = 0.2
            motion.stable_frame = self.srdf_model.get_base_link(group_name)
            motion.segment_duration = 0.0

            for idx in range(len(group_names)) :
                group_name = group_names[idx]
                pt = pts[idx]

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
                initial_region = final_region

                wp = geometry_msgs.msg.PoseStamped()
                wp.pose = pt
                final_region.goal_frame = wp
                final_region.goal_frame.header.stamp = rospy.Time(0)
                final_region.goal_frame.header.frame_id = frame_ids[idx].lstrip("/")
                motion.final_goal_regions.append(final_region)

                initial_region.goal_frame.header.frame_id = self.srdf_model.get_tip_link(group_name).lstrip("/")

                initial_region.goal_frame.header.stamp = rospy.Time(0)
                initial_region.goal_frame.pose.orientation.w = 1.0
                motion.initial_goal_regions.append(initial_region)

        goal.segments.append(motion);

        print goal

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- waiting for result")
        self.cartesian_reach_client.wait_for_result()

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- got result")
        resp = self.cartesian_reach_client.get_result()

        print resp.visualization_plan.trajectory

        return resp.visualization_plan.trajectory



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
        self.srdf_model.set_joint_mask[group_name, mask]

       
if __name__=="__main__":

    rospy.init_node("atlas_planner_client")
    pp = AtlasPathPlanner("atlas", "atlas_moveit_config")
    rospy.spin()
