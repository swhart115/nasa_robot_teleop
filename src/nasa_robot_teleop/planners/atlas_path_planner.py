#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import PyKDL as kdl

roslib.load_manifest('step_finder')
roslib.load_manifest('walk_controller')
roslib.load_manifest('auto_walker')
roslib.load_manifest('reactive_walker')

roslib.load_manifest('matec_msgs')

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg
import std_msgs.msg
from drc_msgs.msg import Pose2D

import actionlib
from actionlib_msgs.msg import GoalStatus
from matec_msgs.msg import *
from matec_actions.msg import *

from walk_controller.msg import *
from auto_walker.msg import *
from reactive_walker.msg import *

from matec_msgs.srv import *
from step_finder.srv import *
from auto_walker.srv import *

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
        self.navigation_frame = rospy.get_param("/interactive_control/navigation_frame", "pelvis")
        self.planning_frame = rospy.get_param("/interactive_control/navigation_frame", "pelvis")
        self.groups = {}
        self.joint_names = []
        self.feet_names = ['left', 'right']
        self.wait_for_service_timeout = 5.0

        rospy.set_param("~atlas/interpolation_type", 1)
        rospy.set_param("~atlas/num_visualizaton_points", 5)
        rospy.set_param("~atlas/visualize_path", True)
        rospy.set_param("~atlas/maintain_hand_pose_offsets", False)
        rospy.set_param("~atlas/move_as_far_as_possible", False)

        rospy.set_param("~atlas/duration", 2.0)
        rospy.set_param("~atlas/allow_incomplete_planning", True)
        rospy.set_param("~atlas/num_acceptable_consecutive_failures", 0)
        rospy.set_param("~atlas/plan_visualization_density", 0.5)
        rospy.set_param("~atlas/visualize_on_plan", True)
        rospy.set_param("~atlas/max_angular_velocity", 0.4)
        rospy.set_param("~atlas/max_linear_velocity", 0.1)

        rospy.set_param("~atlas/navigation_mode", "AUTO_WALKER")
        rospy.set_param("~atlas/plan_footsteps", True)
        rospy.set_param("~atlas/plan_through_unknown_cells", True)
        rospy.set_param("~atlas/assume_flat_ground", True)
        rospy.set_param("~atlas/auto_walker_timeout", 10.0)

        rospy.set_param("~atlas/reactive_walk/success_radius", 0.4)
        rospy.set_param("~atlas/reactive_walk/max_vel", 0.15)
        rospy.set_param("~atlas/reactive_walk/max_accel_steps", 4)
        rospy.set_param("~atlas/reactive_walk/timeout", 0.0)
        rospy.set_param("~atlas/reactive_walk/enqueue", False)

        self.cartesian_reach_client = actionlib.SimpleActionClient('/planned_manipulation/server', matec_actions.msg.PlannedManipulationAction)
        self.joint_action_client = actionlib.SimpleActionClient('/base_joint_interpolator/server', control_msgs.msg.FollowJointTrajectoryAction)
        
        self.walk_controller_client = actionlib.SimpleActionClient('/path_walker', walk_controller.msg.WalkPathAction)
        self.auto_walker_client = actionlib.SimpleActionClient('/auto_walker/autonomous_server', auto_walker.msg.AutonomousWalkAction)
        self.reactive_walker_client = actionlib.SimpleActionClient('/reactive_walk', reactive_walker.msg.ReactiveWalkAction)
        self.explicit_walk_client = actionlib.SimpleActionClient('/auto_walker/explicit_server', auto_walker.msg.ExplicitWalkAction)
        
        self.goal_region_pub = rospy.Publisher(str('/' + self.robot_name + '/planned_path_visualization'), visualization_msgs.msg.MarkerArray, latch=False, queue_size=10)

        # self.planner_feedback_sub = rospy.Subscriber('/planned_manipulation/server/feedback', matec_actions.msg.PlannedManipulationActionFeedback, self.planner_feedback)
        # self.planner_viz_feedback_sub = rospy.Subscriber('/planned_manipulation/plan_visual', control_msgs.msg.FollowJointTrajectoryGoal, self.planner_viz_feedback)
      
        try :
            rospy.loginfo("AtlasPathPlanner::init() -- waiting for visualization service")
            self.connected_to_plan_viz = rospy.wait_for_service('/planned_manipulation/visualize', 3.0)
            if not self.connected_to_plan_viz :
                rospy.logwarn("AtlasPathPlanner::init() -- timeout out waiting for visualization service")  
        except :
            rospy.logwarn("AtlasPathPlanner::init() -- timeout out waiting for visualization service")  

        self.last_plan_name = ""
        self.get_joint_names()

        self.navigation_mode = rospy.get_param("~atlas/navigation_mode")

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
        self.groups[group_name].planning_frame = self.get_robot_planning_frame()
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
        # return self.planning_frame
        parents = self.urdf_model.parent_map.keys()
        links = self.urdf_model.link_map.keys()
        pm = list(set(links) - set(parents))
        return pm[0]

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

    def get_group_joints(self, group_name, fixed=False) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("AtlasPathPlanner::get_group_joints() -- group name \'" + str(group_name) + "\' not found"))
            return []
        else :
            if fixed :
                return self.srdf_model.full_group_joints[group_name]
            else :
                return self.groups[group_name].joint_map.names

    def get_start_foot(self) :
        return rospy.get_param("~atlas/start_foot")

    def set_start_foot(self, foot) :
        rospy.set_param("~atlas/start_foot", foot)


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

    def execute_plans(self, group_names, from_stored=False, wait=True) :

        plan_name = ""       

        for group_name in group_names :
        
            rospy.loginfo(str("AtlasPathPlanner::execute_plans(" + group_name + ")"))

            # hack to send individual foot goal to robot
            if "_leg" in group_name :
                rospy.logwarn("AtlasPathPlanner::execute_plans() -- can't execute leg command directly; use \'execute_on_plan\' instead")
                continue

            plan_name += str("/") + group_name

        req = ExecuteManipulationPlanRequest()
        rospy.logwarn(str("AtlasPathPlanner::execute_plans for " + plan_name ))

        req.plan_names.append(plan_name)
        try :
            executor = rospy.ServiceProxy("/planned_manipulation/execute", ExecuteManipulationPlan)
            resp = executor(req)
            # for p in resp.progress :
            #     rospy.loginfo(str("AtlasPathPlanner::execute_plans(" + group_name + ") progress: " + str(p)))
            return True
        except rospy.ServiceException, e:
            rospy.logwarn(str("AtlasPathPlanner::execute_plans(" + str(plan_name) 
                + ") -- ExecuteCommand service call failed for plan: " + str(plan_name) 
                + " -- possibly a joint plan"))
            return False

    def execute_navigation_plan(self, footsteps=None, lift_heights=None, feet=None, goals=None) :

        rospy.loginfo("AtlasPathPlanner::execute_navigation_plan()")

        ret = False
        mode = rospy.get_param("~atlas/navigation_mode")
        if mode == "WALK_CONTROLLER" :
            ret = self.execute_walk_controller(footsteps, lift_heights, feet)
        elif mode == "AUTO_WALKER" :
            ret = self.execute_auto_walker(footsteps, lift_heights, feet)
        elif mode == "REACTIVE_WALKER" :
            ret = self.execute_reactive_walker(goals)
        else :
            rospy.logerr(str("AtlasPathPlanner::execute_navigation_plan() -- unknown mode " + str(mode)))
          
        return ret 


    def execute_walk_controller(self, footsteps, lift_heights, feet) :

        if not self.walk_controller_client.wait_for_server(rospy.Duration(2.0)) :
            rospy.logerr("AtlasPathPlanner::execute_walk_controller() -- wait for server timeout")
            return False

        # Creates a goal to send to the action server.
        goal = walk_controller.msg.WalkPathGoal()

        # just use config stuff from first plan request until we allow stuff to be different across waypoints/groups
        idx = 0
        for step in footsteps :

            self.tf_listener.waitForTransform("/global", step.header.frame_id, rospy.Time(0), rospy.Duration(5.0))
                
            step = self.tf_listener.transformPose("/global", step)
            p = Pose2D()
            p.x = step.pose.position.x
            p.y = step.pose.position.y

            q = [0]*4
            q[0] = step.pose.orientation.x
            q[1] = step.pose.orientation.y
            q[2] = step.pose.orientation.z
            q[3] = step.pose.orientation.w

            R = kdl.Rotation.Quaternion(q[0],q[1],q[2],q[3])
            rpy = R.GetRPY()
            p.theta = rpy[2] 

            n = geometry_msgs.msg.Vector3(R[0,2],R[1,2],R[2,2])

            goal.path.append(p)
            goal.heights.append(step.pose.position.z)
            goal.normals.append(n)
    
        try : 
            goal.left_foot_start = feet[0] == 0 # FIXME
        except :
            goal.left_foot_start = rospy.get_param("~atlas/start_foot") == "left"
          
        goal.step_duration = 0.6
        goal.step_mode = True
        goal.relative_height = False
        goal.mode = 4
        goal.params_set = False
        goal.linear_stride_length = 0.25
        goal.side_stride_length = 0.02
        goal.rotational_stride_length = 0.01
        goal.stride_width = 0.2
        goal.swing_height = 0.1
        goal.turn_out = 0.0
        goal.final_turn_out = 0.0
        goal.final_stride_width = 0.2
        goal.stamp_in_place_before = False
        goal.stamp_in_place_after = False
        goal.back_up_distance = 0.0

        rospy.loginfo("AtlasPathPlanner::execute_walk_controller() -- sending walk path goal")
        # Sends the goal to the action server.
        self.walk_controller_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        # rospy.loginfo("AtlasPathPlanner::execute_walk_controller() -- waiting for walk path result")
        # self.walk_controller_client.wait_for_result()

        try :
            fb_msg = rospy.wait_for_message("/path_walker/feedback", walk_controller.msg.WalkPathActionFeedback, 3.0)
            while not fb_msg.feedback.planning_complete:
                fb_msg = rospy.wait_for_message("/path_walker/feedback", walk_controller.msg.WalkPathActionFeedback, 3.0)
        except :
            rospy.logwarn("AtlasPathPlanner::execute_walk_controller() -- timeout on sending goal to walk")
        
        rospy.loginfo("AtlasPathPlanner::execute_walk_controller() -- COMPLETE(?)")

        # Prints out the result of executing the action
        # print self.walk_controller_client.get_result()  
        status = self.walk_controller_client.get_state() == GoalStatus.SUCCEEDED

        return status

    def execute_auto_walker(self, footsteps, lift_heights, feet) :

        if not self.explicit_walk_client.wait_for_server(rospy.Duration(2.0)) :
            rospy.logerr("AtlasPathPlanner::execute_auto_walker() -- wait for explicit_walk server timeout")
            return False

        if lift_heights == None or feet == None :
            rospy.logerr("AtlasPathPlanner::execute_auto_walker() -- didnt get lift heights or feet indices")
            return False

        if len(footsteps) != len(lift_heights) or len(footsteps) != len(feet) :
            rospy.logerr("AtlasPathPlanner::execute_auto_walker() -- size mismatch for footsteps info")
            return False

        # Creates a goal to send to the action server.
        goal = auto_walker.msg.ExplicitWalkGoal()
        goal.execute_immediately = True
        goal.step_poses = copy.deepcopy(footsteps)
        goal.feet = copy.deepcopy(feet)
        goal.lift_heights = copy.deepcopy(lift_heights)

        rospy.loginfo("AtlasPathPlanner::execute_auto_walker() -- sending goal")
        self.explicit_walk_client.send_goal(goal)

        # try :
        #     fb_msg = rospy.wait_for_message("/auto_walker/explicit_server", auto_walker.msg.ExplicitWalkActionFeedback, 3.0)
        #     while not fb_msg.feedback.progress:
        #         fb_msg = rospy.wait_for_message("/path_walker/feedback", auto_walker.msg.ExplicitWalkActionFeedback, 3.0)
        # except :
        #     rospy.logwarn("AtlasPathPlanner::execute_auto_walker() -- timeout on sending goal to walk")
        
        rospy.loginfo("AtlasPathPlanner::execute_auto_walker() -- COMPLETE(?)")

        # # Prints out the result of executing the action
        # # print self.explicit_walk_client.get_result()  
        # status = self.explicit_walk_client.get_state() == GoalStatus.SUCCEEDED

        status = self.explicit_walk_client.get_state() == GoalStatus.SUCCEEDED

        return status


    def execute_reactive_walker(self, goals) :

        if len(goals) > 1 :
            rospy.logwarn("AtlasPathPlanner::execute_reactive_walker() -- too many goals, taking last")
            goal_pose = goals[len(goals)-1]
        else :
            goal_pose = goals[0]

        if not self.reactive_walker_client.wait_for_server(rospy.Duration(2.0)) :
            rospy.logerr("AtlasPathPlanner::execute_reactive_walker() -- wait for reactive_walk server timeout")
            return False

        # Creates a goal to send to the action server.
        goal = reactive_walker.msg.ReactiveWalkGoal()

        pose = geometry_msgs.msg.Pose2D()
        pose.x = goal_pose.pose.position.x
        pose.y = goal_pose.pose.position.y

        q = [goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w]
        r,p,y = q_to_rpy(q)
        pose.theta = y
        
        goal.frame_id = goal_pose.header.frame_id
        goal.goal = pose  
        goal.success_radius = rospy.get_param("~atlas/reactive_walk/success_radius")
        goal.enqueue = rospy.get_param("~atlas/reactive_walk/enqueue")
        goal.timeout = rospy.Duration(rospy.get_param("~atlas/reactive_walk/timeout"))
        goal.max_vel = rospy.get_param("~atlas/reactive_walk/max_vel")
        goal.max_accel_steps = rospy.get_param("~atlas/reactive_walk/max_accel_steps")

        rospy.loginfo("AtlasPathPlanner::execute_reactive_walker() -- sending goal")
        self.reactive_walker_client.send_goal(goal)

        return True

    def direct_move(self, goal) :
        try:
            self.execute_reactive_walker([goal])
        except Exception as e:
            print e
            rospy.logerr("AtlasPathPlanner::direct_move() -- problems")

    def get_navigation_modes(self) :
        return ["WALK_CONTROLLER", "AUTO_WALKER", "REACTIVE_WALKER"]

    def get_navigation_mode(self) :
        return self.navigation_mode

    def set_navigation_mode(self, mode) :
        if mode in self.get_navigation_modes() :
            self.navigation_mode = mode
            rospy.set_param("~atlas/navigation_mode", mode)

    def accommodate_terrain_in_navigation(self) :
        return not rospy.get_param("~atlas/assume_flat_ground")

    def set_accommodate_terrain_in_navigation(self, val) :
        rospy.set_param("~atlas/assume_flat_ground", not bool(val))

    ##################################
    ######## PLANNING METHODS ########
    ##################################
  
    def plan_joint_goals(self, group_names, goals) :

        # assumes each element of goals is an array of JointState() msgs representing a path for the
        # corresponding index of group_names
        
        rospy.loginfo("AtlasPathPlanner::plan_joint_goals()")

        if not len(group_names) == len(goals) :
            rospy.logerr(str("AtlasPathPlanner::plan_joint_goals() -- size mismatch: " 
                + str(len(group_names)) + " groups vs. " + str(len(goals)) + " goals"))
            return None 

        # check non-overlapping joint names within the groups
        joint_names = []
        idx = 0
        for group in group_names :
            # goal = goals[idx]
            # if self.group_types[group] == "endeffector" :
            #     self.create_joint_plan_from_goal(group, goal)
            group_joints = self.srdf_model.full_group_joints[group]
            for j in group_joints :
                if j in joint_names :
                    rospy.logerr(str("AtlasPathPlanner::plan_joint_goals() -- overlapping joint (" + j + ") in joint goals. confused."))
                    return None 
                else :
                    joint_names.append(j)

        # reshape goals into paths
        paths = []
        for g in goals :
            paths.append([g])

        return self.plan_joint_paths(group_names, paths)


    def plan_joint_paths(self, group_names, paths) :

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
        num_segments = len(paths[0]) # all path lengths for each group should match
        num_groups = len(group_names)

        segments = []
        for idx in range(num_segments) :
            segment = []
            for jdx in range(num_groups) :
                group_name = group_names[jdx]
                segment.append(paths[jdx][idx])
                for n in paths[jdx][idx].name :
                    goal.trajectory.joint_names.append(n)               
            segments.append(segment)

        for segment in segments :
            p = trajectory_msgs.msg.JointTrajectoryPoint()          
            for j in segment :
                p.positions = j.position    
            goal.trajectory.points.append(p)
       
        rospy.loginfo("AtlasPathPlanner::plan_joint_paths() -- waiting for server")
        if not self.joint_action_client.wait_for_server(rospy.Duration(2.0)) :
            rospy.logerr("AtlasPathPlanner::plan_joint_paths() -- wait for server timeout")
            return False

        # Creates a goal to send to     the action server.
        rospy.loginfo("AtlasPathPlanner::plan_joint_paths() -- sending goal")

        # Sends the goal to the action server.
        self.joint_action_client.send_goal(goal)

        # # Waits for the server to finish performing the action.
        # rospy.loginfo("AtlasPathPlanner::plan_joint_paths() -- waiting for result")
        # self.joint_action_client.wait_for_result()

        return goal.trajectory
        

    def plan_cartesian_goals(self, group_names, goals) :
        
        # assumes each element of goal is a PoseStamped() msgs representing the desired goal
        # for the corresponding index of group_names

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_goals()")

        if len(group_names) != len(goals) :
            rospy.logerr(str("AtlasPathPlanner::plan_cartesian_goals() -- size mismatch: " 
                + str(len(group_names)) + " groups vs. " + str(len(goals)) + " goals"))
            return None

        # reshape goals into paths
        paths = []
        for g in goals :
            paths.append([g])

        return self.plan_cartesian_goals(group_names, paths)


    def plan_cartesian_paths(self, group_names, paths) :
        
        # assumes each element of paths is an array of PoseStamped() msgs representing a path for the
        # corresponding index of group_names

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths()")

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- waiting for server")
        if not self.cartesian_reach_client.wait_for_server(rospy.Duration(2.0)) :
            rospy.logerr("AtlasPathPlanner::plan_cartesian_paths() -- wait for server timeout")
            return False

        goal = matec_actions.msg.PlannedManipulationGoal()
        goal.visualize_on_plan = rospy.get_param("~atlas/visualize_on_plan")
        goal.allow_incomplete_planning = rospy.get_param("~atlas/allow_incomplete_planning")
        goal.num_acceptable_consecutive_failures = rospy.get_param("~atlas/num_acceptable_consecutive_failures")
        goal.plan_visualization_density = rospy.get_param("~atlas/plan_visualization_density")
        goal.execute_on_plan = True in [self.auto_execute[g] for g in group_names]
       
        goal.plan_name = ""
        for g in group_names :
            goal.plan_name += str("/") + g

        self.last_plan_name = goal.plan_name
        rospy.logwarn("Setting plan_name: " + self.last_plan_name)

        num_groups = len(group_names)
        num_segments = len(paths[0])

        segments = []
        for idx in range(num_segments) :
            segment = []
            for jdx in range(num_groups) :
                segment.append(paths[jdx][idx])
            segments.append(segment)

        for segment in segments :

            motion = matec_msgs.msg.GoalMotion()
            motion.max_angular_velocity = rospy.get_param("~atlas/max_angular_velocity")
            motion.max_linear_velocity = rospy.get_param("~atlas/max_linear_velocity")
            motion.stable_frame = self.get_robot_planning_frame()
            motion.segment_duration = 0.0
            motion.available_joints = []

            idx = 0
            for wp in segment :

                group_name = group_names[idx]

                group_joints = self.get_group_joints(group_name)
                masked_group_joints = []
                mask = self.get_joint_mask(group_name)
                for jdx in range(len(group_joints)) :
                    if mask[jdx] :
                        masked_group_joints.append(group_joints[jdx])
                joints = list(set(motion.available_joints + masked_group_joints))
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

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- sending goal")
        # Sends the goal to the action server.
        self.cartesian_reach_client.send_goal(goal)

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- polling feedback")
        fb_msg = rospy.wait_for_message("/planned_manipulation/server/feedback", matec_actions.msg.PlannedManipulationActionFeedback, 3.0)

        while not fb_msg.feedback.planning_complete:
            fb_msg = rospy.wait_for_message("/planned_manipulation/server/feedback", matec_actions.msg.PlannedManipulationActionFeedback, 3.0)

        rospy.loginfo("AtlasPathPlanner::plan_cartesian_paths() -- PLANNING COMPLETE(?)")
        if fb_msg.feedback.planning_progress > 0.0 :            
            p = self.get_plan()
            return p
        else :
            return None


    def plan_navigation_path(self, waypoints) :

        mode = rospy.get_param("~atlas/navigation_mode")
        steps = None
        lift_heights = None
        feet = None
        if mode == "WALK_CONTROLLER" :
            steps = self.plan_path_walk_controller(waypoints)
        elif mode == "AUTO_WALKER" :
            steps, lift_heights, feet = self.plan_path_auto_walker(waypoints)
        elif mode == "REACTIVE_WALKER" :
            rospy.logwarn("AtlasPathPlanner::plan_navigation_path() -- no sense planning in REACTIVE_WALKER mode ")
        else :
            rospy.logerr(str("AtlasPathPlanner::plan_navigation_path() -- unknown mode " + str(mode)))
            
        return steps, lift_heights, feet

        
    def plan_path_auto_walker(self, waypoints) :

        # send auto walk action goal, wait for planning complete, call service to get steps

        # if i like, them call execute service, if not, modify, send to explicit

        step_poses = []
        lift_heights = []
        feet = []


        rospy.loginfo("AtlasPathPlanner::plan_path_auto_walker() -- waiting for auto_walker server")
        if not self.auto_walker_client.wait_for_server(rospy.Duration(2.0)) :
            rospy.logerr("AtlasPathPlanner::plan_path_auto_walker() -- wait for auto_walker server timeout")
            return step_poses, lift_heights, feet

        goal = auto_walker.msg.AutonomousWalkGoal()
        goal.targets = copy.deepcopy(waypoints)
        goal.execute_on_plan = False
        goal.assume_flat_ground = rospy.get_param("~atlas/assume_flat_ground")
        goal.plan_through_unknown_cells = rospy.get_param("~atlas/plan_through_unknown_cells")
        goal.solver_timeout = rospy.get_param("~atlas/auto_walker_timeout")

        rospy.loginfo("AtlasPathPlanner::plan_path_auto_walker() -- sending goal")
        # Sends the goal to the action server.
        self.auto_walker_client.send_goal(goal)

        try :
            rospy.loginfo("AtlasPathPlanner::plan_path_auto_walker() -- polling feedback")
            fb_msg = rospy.wait_for_message("/auto_walker/autonomous_server/feedback", auto_walker.msg.AutonomousWalkFeedback, 5.0)

            while not fb_msg.feedback.planning_complete:
                fb_msg = rospy.wait_for_message("/planned_manipulation/server/feedback", auto_walker.msg.AutonomousWalkFeedback, 5.0)
            rospy.loginfo("AtlasPathPlanner::plan_path_auto_walker() -- planning complete, getting resulting steps...")

            try :
                rospy.wait_for_service("/auto_walker/get_step_plan", self.wait_for_service_timeout)

                try :
                    rospy.loginfo("AtlasPathPlanner::plan_path_auto_walker() -- requesting plan!")
                    step_planner = rospy.ServiceProxy("/auto_walker/get_step_plan", auto_walker.srv.GetStepPlan)
                    resp = step_planner()

                    step_poses = resp.step_poses
                    lift_heights = resp.lift_heights
                    feet = resp.feet

                    rospy.loginfo("AtlasPathPlanner::plan_path_auto_walker() -- got footsteps!")

                except rospy.ServiceException, e:
                    rospy.logerr(str("AtlasPathPlanner::plan_path_auto_walker() -- faild to get step service" + str(e)))

            except rospy.ROSException as e:
                rospy.logerr("AtlasPathPlanner::plan_path_auto_walker() -- faile to wait for step service: " + str(e))

        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::plan_path_auto_walker() -- failed to get feedback message: " + str(e))

        return step_poses, lift_heights, feet


    def plan_path_walk_controller(self, waypoints) :

        rospy.loginfo("AtlasPathPlanner::plan_path_walk_controller()")
        
        req = PlanStepsRequest()

        for wp in waypoints :
            req.target.append(wp)
            
        req.plan_through_unknown_cells = True
        req.solver_timeout = 10.0

        try :
            rospy.wait_for_service("/plan_steps", self.wait_for_service_timeout)
        except rospy.ROSException as e:
            rospy.logerr("AtlasPathPlanner::plan_path_walk_controller(): " + str(e))
            return None

        try :
            rospy.loginfo("AtlasPathPlanner::plan_path_walk_controller() -- requesting plan!")
            step_planner = rospy.ServiceProxy("/plan_steps", PlanSteps)
            resp = step_planner(req)
        except rospy.ServiceException, e:
            rospy.logerr(str("AtlasPathPlanner::plan_path_walk_controller() -- " + str(e)))
            return None

        rospy.loginfo("AtlasPathPlanner::plan_path_walk_controller() -- got footsteps!")

        if resp.left_foot_start :            
            rospy.set_param("~atlas/start_foot", "left")
        else :
            rospy.set_param("~atlas/start_foot", "right")
        
        return resp.steps

    def plan_path_reactive_walker(self, waypoints) :
        rospy.logwarn("AtlasPathPlanner::plan_path_reactive_walker() -- not implemented!")
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
            jt = self.get_plan() 
            self.process_plan_viz(jt)

    def planner_viz_feedback(self, data) :
        jt = data.trajectory
        self.process_plan_viz(jt)

    def get_plan(self) :    

        try:
            rospy.wait_for_service('/planned_manipulation/visualize', 3.0)
            req = VisualizeManipulationPlanRequest()
            req.plan_names.append(self.last_plan_name)
            req.plans_in_parallel = False
            req.plan_visualization_density = rospy.get_param("~atlas/plan_visualization_density")
            get_plan = rospy.ServiceProxy('/planned_manipulation/visualize', VisualizeManipulationPlan)
            resp = get_plan(req)
        except :
            rospy.logerr("AtlasPathPlanner::get_plan_viz() -- service call failed")
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
