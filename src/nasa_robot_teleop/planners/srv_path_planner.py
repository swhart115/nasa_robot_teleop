#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

roslib.load_manifest('step_finder')

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg

from step_finder.srv import *

from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.util.kinematics_util import *

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *


class SrvPathPlanner(PathPlanner) :

    ############################
    ####### CONSTRUCTOR ########
    ############################

    def __init__(self, robot_name, config_package, namespace="/interactive_controls_bridge"):
        PathPlanner.__init__(self, robot_name, config_package)
        rospy.loginfo(str("============ Setting up Path Planner for: \'" + self.robot_name + "\'"))
        self.planning_frame = rospy.get_param("~navigation_frame", "/global")
        self.groups = {}
        self.joint_names = []
        self.feet_names = ['left', 'right']
        self.namespace = namespace

        rospy.set_param("~interpolation_type", 1)
        rospy.set_param("~duration", 2.0)
        rospy.set_param("~num_visualizaton_points", 1)
        rospy.set_param("~visualize_path", True)
        rospy.set_param("~maintain_hand_pose_offsets", False)
        rospy.set_param("~move_as_far_as_possible", False)

        # self.load_configurations()
        rospy.loginfo(str("============ Setting up Path Planner for robot: \'" + self.robot_name + "\' finished"))
        
    ##############################
    ####### SETUP METHODS ########
    ##############################

    def load_configurations(self) :
        req = GetPlanningServiceConfigurationRequest()
        rospy.wait_for_service(str(self.namespace + "/get_config"))
        try :
            get_planner_config = rospy.ServiceProxy(str(self.namespace + "/get_config", GetPlanningServiceConfiguration))
            resp = get_planner_config(req)
            for g in resp.group_configurations:
                self.groups[g.group_name] = g
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::load_configurations() -- GetPlanningServiceConfiguration service call failed: " + str(e)))

    def configure_group(self, group_name) :
        r = True
        rospy.loginfo(str("SrvPathPlanner::configure_group() -- " + group_name))
        if not group_name in self.groups :
            rospy.logerr(str("SrvPathPlanner::configure_group(" + group_name + ") -- group not found"))
            return False
        req = ConfigurePlanningServiceRequest()
        rospy.wait_for_service(str(self.namespace + "/config"))
        try :
            req.group_configurations.append(self.groups[group_name])
            configure_planner = rospy.ServiceProxy(str(self.namespace + "/config", ConfigurePlanningService))
            resp = configure_planner(req)
            if not resp.status :
                rospy.logwarn(str("SrvPathPlanner::configure_group(" + group_name + ") -- status error"))
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::configure_group() -- Service call failed: " + str(e)))
            r = False
        return r

    def setup_group(self, group_name, joint_tolerance, position_tolerances, orientation_tolerances) :
        rospy.loginfo(str("SrvPathPlanner::setup_group() -- " + group_name))
        self.position_tolerances[group_name] = position_tolerances
        self.orientation_tolerances[group_name] = orientation_tolerances
        self.joint_tolerance[group_name] = joint_tolerance
        return self.load_group_from_srdf(group_name)

    def load_group_from_srdf(self, group_name) :

        if not group_name in self.srdf_model.groups :
            return False

        rospy.loginfo(str("SrvPathPlanner::load_group_from_srdf() -- group: " + group_name))

        self.groups[group_name] = PlanGroupConfiguration()
        self.groups[group_name].joint_map = self.lookup_joint_map(group_name)

        N = len(self.groups[group_name].joint_map.names)

        if N==0 :
            rospy.logerr("SrvPathPlanner::load_group_from_srdf() -- no joint names found in the map!")
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

        self.get_joint_names()

        for j in joint_list :
            if not j in self.joint_names :
                continue
            joint_name_map.names.append(j)
            joint_name_map.ids.append(self.joint_names.index(j))

        return joint_name_map

    def get_joint_names(self) :
        rospy.wait_for_service(str(self.namespace + "/get_joint_names"))
        try :
            rospy.loginfo(str("SrvPathPlanner::get_joint_names() -- calling service"))
            get_names = rospy.ServiceProxy(str(self.namespace + "/get_joint_names", GetJointNames))
            resp = get_names()
            self.joint_names = resp.joint_names
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::get_joint_names()" + str(e)))
            return None

    #################################
    ####### OBSTACLE METHODS ########
    #################################

    def add_obstacle(self, p, s, n) :
        rospy.logerr("SrvPathPlanner::add_obstacle() -- obstacle avoidance not supported")


    ################################
    ######## HELPER METHODS ########
    ################################

    def get_robot_planning_frame(self) :
        return self.planning_frame

    def get_group_planning_frame(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("SrvPathPlanner::get_group_planning_frame() -- group name \'" + str(group_name) + "\' not found"))
            return ""
        else :
            return self.groups[group_name].planning_frame

    def has_end_effector_link(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.loginfo(str("SrvPathPlanner::has_end_effector_link() -- group name \'" + str(group_name) + "\' not found"))
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
            rospy.logerr(str("SrvPathPlanner::get_end_effector_link() -- group name \'" + str(group_name) + "\' not found"))
            return ""
        else :
            return self.groups[group_name].control_frame

    def clear_goal_target(self, group_name) :
        pass

    def get_group_joints(self, group_name) :
        if not group_name in self.groups.keys() :
            rospy.logerr(str("SrvPathPlanner::get_group_joints() -- group name \'" + str(group_name) + "\' not found"))
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
        rospy.loginfo(str("SrvPathPlanner::execute_plan(" + group_name+ ")"))

        req = ExecuteCommandRequest()
        req.groups.append(group_name)

        rospy.wait_for_service(str(self.namespace + "/execute_command"))
        try :
            executor = rospy.ServiceProxy(str(self.namespace + "/execute_command", ExecuteCommand))
            resp = executor(req)
            for p in resp.progress :
                rospy.loginfo(str("SrvPathPlanner::execute_plan(" + group_name + ") progress: " + str(p)))
            return True
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::execute_plan(" + group_name + ") -- ExecuteCommand service call failed: " + str(e)))
            return False

    def execute_navigation_plan(self, footsteps) :

        rospy.loginfo("SrvPathPlanner::execute_navigation_plan()")
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
            rospy.logerr(str("SrvPathPlanner::execute_navigation_plan()"))
            return None

        # print "==================================================="
        # print "Footstep plan:"
        # print req
        # print "==================================================="
        
        rospy.wait_for_service(str(self.namespace + "/navigation_plan_command"))
        try :
            rospy.loginfo(str("SrvPathPlanner::execute_navigation_plan() -- calling service"))
            planner = rospy.ServiceProxy(str(self.namespace + "/navigation_plan_command", CartesianPlanCommand))
            resp = planner(req)

            if len(resp.result) > 0 :
                return resp.result[0]
            else :
                rospy.logwarn(str("SrvPathPlanner::execute_navigation_plan() -- failed to get footstep plan to goal"))
                return None
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::execute_navigation_plan()" + str(e)))
            return None

    def multigroup_execute_plan(self, group_names, from_stored=False, wait=True) :
        r = []
        for g in group_names:
            r.append(self.execute_plan(g,from_stored,wait))
        return r


    ##################################
    ######## PLANNING METHODS ########
    ##################################

    def plan_to_cartesian_goal(self, group_name, pt) :
        
        rospy.loginfo("SrvPathPlanner::plan_to_cartesian_goal()")

        req = CartesianPlanCommandRequest()
        
        interpolation_type = rospy.get_param("~interpolation_type")
        duration = rospy.get_param("~duration")
        num_visualizaton_points = rospy.get_param("~num_visualizaton_points")

        req.return_trajectories = rospy.get_param("~visualize_path")
        req.maintain_hand_pose_offsets = rospy.get_param("~maintain_hand_pose_offsets")
        req.move_as_far_as_possible = rospy.get_param("~move_as_far_as_possible")

        if group_name in self.groups.keys() :

            req.execute_on_plan = self.auto_execute[group_name]

            got = self.get_goal_orientation_tolerances(group_name)
            gpt = self.get_goal_position_tolerances(group_name)

            gotv = geometry_msgs.msg.Vector3(got[0],got[1],got[2])
            gptv = geometry_msgs.msg.Vector3(gpt[0],gpt[1],gpt[2])

            spec = CartesianPlanRequestSpecification()
            spec.group_id = self.groups[group_name].group_id
            spec.group_name = self.groups[group_name].group_name
            spec.joint_mask = self.groups[group_name].joint_mask
            spec.control_frame = self.groups[group_name].control_frame
            spec.waypoints.append(pt)
            spec.duration.append(duration)
            spec.num_visualizaton_points = num_visualizaton_points
            spec.angle_variance.append(gotv)
            spec.maximum_angle_variance.append(gotv)
            spec.position_variance.append(gptv)
            spec.maximum_position_variance.append(gptv)
            spec.interpolation_type = interpolation_type
            req.group_plan_specs.append(spec)

        else :
            rospy.logerr(str("SrvPathPlanner::plan_cartesian_path(" + group_name + ") -- no group found of that name!"))
            return None

        rospy.wait_for_service(str(self.namespace + "/cartesian_plan_command"))
        try :
            planner = rospy.ServiceProxy(str(self.namespace + "/cartesian_plan_command", CartesianPlanCommand))
            resp = planner(req)
            if len(resp.result) > 0 :
                return resp.result[0]
            else :
                rospy.logwarn(str("SrvPathPlanner::plan_cartesian_path(" + group_name + ") -- failed to get plan to goal"))
                return None
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- CartesianPlanCommand service call failed: " + str(e)))
            return None

    def plan_to_joint_goal(self, group_name, js) :

        rospy.loginfo("SrvPathPlanner::plan_to_joint_goal()")

        if self.group_types[group_name] == "endeffector" :
            plan = trajectory_msgs.msg.JointTrajectory()
            point = trajectory_msgs.msg.JointTrajectoryPoint()

            plan.header.stamp = rospy.Time.now()
            for j in range(len(js.name)) :
                plan.joint_names.append(js.name[j])
                point.positions.append(js.position[j])
            plan.points.append(point)
            return plan

        req = JointPlanCommandRequest()

        num_visualizaton_points = rospy.get_param("~num_visualizaton_points")
        duration = rospy.get_param("~duration")

        req.return_trajectories = rospy.get_param("~visualize_path")

        if group_name in self.groups.keys() :
            req.execute_on_plan = self.auto_execute[group_name]

            spec = JointPlanRequestSpecification()
            spec.num_visualizaton_points = num_visualizaton_points

            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            goal.trajectory = trajectory_msgs.msg.JointTrajectory()
            goal.trajectory.header = js.header
            goal.trajectory.joint_names = js.name

            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = js.position
            point.velocities = js.velocity
            point.time_from_start = rospy.Duration(duration)
            goal.trajectory.points.append(point)

            for n in js.name :
                tol = control_msgs.msg.JointTolerance()
                tol.name = n
                tol.position = self.joint_tolerance[group_name]
                goal.path_tolerance.append(tol)
                goal.goal_tolerance.append(tol)

            spec.waypoints.append(goal)
            req.group_plan_specs.append(spec)

        rospy.wait_for_service(str(self.namespace + "/joint_plan_command"))
        try :
            planner = rospy.ServiceProxy(str(self.namespace + "/joint_plan_command", JointPlanCommand))
            resp = planner(req)
            return resp.result[0]
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::plan_to_joint_goal(" + group_name + ") -- JointPlanCommand service call failed: " + str(e)))
            return None

    def plan_to_random_goal(self, group_name) :
        rospy.loginfo("SrvPathPlanner::plan_to_random_goal()")
        
        try :
            self.groups[group_name].set_random_target() # FIXME
            plan = self.groups[group_name].plan()  # FIXME
            return plan.joint_trajectory
        except :
            rospy.logwarn(str("SrvPathPlanner::plan_to_random_goal(" + group_name + ") -- failed"))
            return None

    def plan_cartesian_path(self, group_name, waypoints) :

        rospy.loginfo("SrvPathPlanner::plan_cartesian_path()")
        req = CartesianPlanCommandRequest()

        interpolation_type = rospy.get_param("~interpolation_type")
        duration = rospy.get_param("~duration")
        num_visualizaton_points = rospy.get_param("~num_visualizaton_points")

        req.return_trajectories = rospy.get_param("~visualize_path")
        req.maintain_hand_pose_offsets = rospy.get_param("~maintain_hand_pose_offsets")
        req.move_as_far_as_possible = rospy.get_param("~move_as_far_as_possible")

        if group_name in self.groups.keys() :

            req.execute_on_plan = self.auto_execute[group_name]

            spec = CartesianPlanRequestSpecification()
            spec.group_id = self.groups[group_name].group_id
            spec.group_name = self.groups[group_name].group_name
            spec.joint_mask = self.groups[group_name].joint_mask
            spec.control_frame = self.groups[group_name].control_frame

            for wp in waypoints :

                ps = geometry_msgs.msg.PoseStamped() 
                ps.pose = wp
                ps.header.stamp = rospy.Time.now()
                ps.header.frame_id = self.get_group_planning_frame(group_name)
                got = self.get_goal_orientation_tolerances(group_name)
                gpt = self.get_goal_position_tolerances(group_name)
                gotv = geometry_msgs.msg.Vector3(got[0],got[1],got[2])
                gptv = geometry_msgs.msg.Vector3(gpt[0],gpt[1],gpt[2])

                spec.waypoints.append(ps)
                spec.duration.append(duration)
                spec.angle_variance.append(gotv)
                spec.maximum_angle_variance.append(gotv)
                spec.position_variance.append(gptv)
                spec.maximum_position_variance.append(gptv)

            spec.interpolation_type = interpolation_type
            spec.num_visualizaton_points = num_visualizaton_points
            req.group_plan_specs.append(spec)

        else :
            rospy.logerr(str("SrvPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- no group found of that name!"))
            return False

        rospy.wait_for_service(str(self.namespace + "/cartesian_plan_command"))
        try :
            planner = rospy.ServiceProxy(str(self.namespace + "/cartesian_plan_command", CartesianPlanCommand))
            resp = planner(req)
            print "cartesian plan got response: "
            print resp
            return resp.result[0]
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::plan_to_cartesian_goal(" + group_name + ") -- PlanCommand service call failed: " + str(e)))
            return None

    def plan_navigation_path(self, waypoints) :

        rospy.loginfo("SrvPathPlanner::plan_navigation_path()")
        
        req = PlanStepsRequest()

        for wp in waypoints :
            req.target.append(wp)
            
        req.plan_through_unknown_cells = True
        req.solver_timeout = 10.0

        rospy.wait_for_service(str(self.namespace + "/plan_steps"))
        try :
            rospy.loginfo("SrvPathPlanner::plan_navigation_path() -- requesting plan!")
            step_planner = rospy.ServiceProxy(str(self.namespace + "/plan_steps", PlanSteps))
            resp = step_planner(req)
        except rospy.ServiceException, e:
            rospy.logerr(str("SrvPathPlanner::plan_navigation_path() -- " + str(e)))
            return None

        rospy.loginfo("SrvPathPlanner::plan_navigation_path() -- got footsteps!")

        if resp.left_foot_start :            
            rospy.set_param("~start_foot", "left")
        else :
            rospy.set_param("~start_foot", "right")
        
        return resp.steps


    ### multigroup functions
    def plan_to_cartesian_goals(self, group_names, pts) :
        r = []
        if not len(group_names) == len(pts) :
            rospy.logerr("SrvPathPlanner::plan_to_cartesian_goals() -- input arg size mismatch")
            r.append(False)
        else :
            for i in len(group_names) :
                r.append(self.plan_to_cartesian_goal(group_names[i], pts[i]))
        return r

    def plan_to_joint_goals(self, group_names, jss) :
        r = []
        if not len(group_names) == len(jss) :
            rospy.logerr("SrvPathPlanner::plan_to_joint_goals() -- input arg size mismatch")
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
            rospy.logerr("SrvPathPlanner::plan_cartesian_paths() -- input arg size mismatch")
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

    def get_joint_mask(self, group_name) :
        return self.groups[group_name].joint_mask.mask

    def set_joint_mask(self, group_name, mask) :
        self.groups[group_name].joint_mask.mask = mask
        self.srdf_model.set_joint_mask[group_name, mask]

if __name__=="__main__":

    rospy.init_node("srv_planner_client")
    pp = SrvPathPlanner("srv", "atlas_moveit_config")
    rospy.spin()
