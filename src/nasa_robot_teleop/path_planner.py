#! /usr/bin/env python

import rospy

import copy
import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')
from rospkg import RosPack

import tf
import PyKDL as kdl

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg
import actionlib_msgs.msg

from nasa_robot_teleop.srv import *

from srdf_model import SRDFModel
from kdl_posemath import *
import urdf_parser_py as urdf
from urdf_helper import *
import end_effector_helper as end_effector


class PathPlanner(object):

    def __init__(self, robot_name, config_package):
        self.robot_name = robot_name
        self.config_package = config_package

        self.robot_name = robot_name
        self.group_types = {}
        self.group_controllers = {}
        self.base_frames = {}
        self.control_frames = {}
        self.control_meshes = {}
        self.control_offset = {}
        self.group_id_offset = {}
        self.end_effector_map = {}
        self.display_modes = {}
        self.trajectory_poses = {}
        self.trajectory_display_markers = {}
        self.end_effector_display = {}
        self.plan_generated = {}
        self.marker_store = {}
        self.stored_plans = {}
        self.config_package = config_package
        self.command_topics = {}
        self.gripper_service = None
        self.bridge_topic_map = {}
        self.actionlib = False

        self.plan_color = (0.5,0.1,0.75,1)
        self.path_increment = 2

        self.path_visualization = rospy.Publisher(str('/' + self.robot_name + '/move_group/planned_path_visualization'), visualization_msgs.msg.MarkerArray, latch=False, queue_size=10)
        self.joint_state_sub = rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)
        self.tf_listener = tf.TransformListener()
  
        if not self.create_models(config_package) :
            rospy.logerr("PathPlanner::init() -- failed creating RDF models")
            return
     
        # for g in self.robot.get_group_names() :
        for g in self.get_group_names() :
            # self.trajectory_publishers[g] = rospy.Publisher(str('/' + self.robot_name + '/' + g + '/move_group/display_planned_path'), moveit_msgs.msg.DisplayTrajectory, queue_size=10)
            self.plan_generated[g] = False
            self.stored_plans[g] = None
            self.display_modes[g] = "last_point"
        

    def get_group_names(self) :
        return self.group_types.keys()

    def has_group(self, group_name) :
        return group_name in self.get_group_names()

    def use_actionlib(self, v) :
        self.actionlib = v

    def set_gripper_service(self, srv) :
        rospy.loginfo("PathPlanner::set_gripper_service() -- set_gripper_service(" + srv + ")")
        rospy.wait_for_service(srv)
        self.gripper_service = rospy.ServiceProxy(srv, EndEffectorCommand)
       
    def clear_gripper_service(self) :
        self.gripper_service = None    

    def create_models(self, config_package) :

        rospy.loginfo("PathPlanner::create_models() -- Creating Robot Model from URDF....")
        self.urdf_model = urdf.Robot.from_parameter_server()
        if self.urdf_model == None : return False

        self.urdf_model.get_all_tips()
        rospy.loginfo("PathPlanner::create_models() -- Creating Robot Model from SRDF....")
        self.srdf_model = SRDFModel(self.robot_name)

        try :
            rospy.loginfo(str("PathPlanner::create_models() -- PathPlanner config package: " + config_package))
            srdf_filename = str(RosPack().get_path(config_package) + "/config/" + self.robot_name + ".srdf")
            rospy.loginfo(str("PathPlanner::create_models() -- SRDF Filename: " + srdf_filename))
            if self.srdf_model.parse_from_file(srdf_filename) :
                rospy.loginfo("PathPlanner::create_models() ================================================")
            return True
        except :
            rospy.logerr("PathPlanner()::create_models() -- error parsing SRDF from file")
            return False

    # problaby can clean this up, it's messy FIXME
    def add_planning_group(self, group_name, group_type="manipulator", joint_tolerance=0.05, position_tolerance=.005, orientation_tolerance=.02) :
        
        # virtual function
        self.setup_group(group_name, joint_tolerance, position_tolerance, orientation_tolerance)

        rospy.loginfo(str("PathPlanner::add_planning_group() -- " + group_name))
        try :
            self.group_types[group_name] = group_type
            self.control_frames[group_name] = ""
            self.control_meshes[group_name] = ""
            self.marker_store[group_name] = visualization_msgs.msg.MarkerArray()
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

            # generate a random group_id
            id_found = False
            while not id_found :
                r = int(random.random()*10000000)
                if not r in self.group_id_offset.values() :
                    self.group_id_offset[group_name] = r
                    id_found = True

            # check to see if the group has an associated end effector, and add it if so
            if self.has_end_effector_link(group_name) :
                self.control_frames[group_name] = self.get_end_effector_link(group_name)
                ee_link = self.urdf_model.link_map[self.get_end_effector_link(group_name)]
                try :
                    # self.control_meshes[group_name] = ee_link.visual.geometry.filename
                    self.control_meshes[group_name] = self.get_child_mesh(ee_link)
                except :
                    rospy.logwarn("no mesh found")
                for ee in self.srdf_model.end_effectors.keys() :
                    if self.srdf_model.end_effectors[ee].parent_group == group_name :
                        self.end_effector_map[group_name] = ee
                        self.add_planning_group(self.srdf_model.end_effectors[ee].group, group_type="endeffector",
                            joint_tolerance=joint_tolerance, position_tolerance=position_tolerance, orientation_tolerance=orientation_tolerance)
            elif self.srdf_model.has_tip_link(group_name) :
                self.control_frames[group_name] = self.srdf_model.get_tip_link(group_name)
                ee_link = self.urdf_model.link_map[self.srdf_model.get_tip_link(group_name)]
                self.control_meshes[group_name] = ee_link.visual.geometry.filename
            elif self.group_types[group_name] == "endeffector" :
                self.control_frames[group_name] = self.srdf_model.group_end_effectors[group_name].parent_link
                ee_link = self.urdf_model.link_map[self.control_frames[group_name]]
                try :
                    self.control_meshes[group_name] = ee_link.visual.geometry.filename
                except :
                    self.control_meshes[group_name] = ""
                    rospy.logwarn("no mesh found")              
                self.end_effector_display[group_name] = end_effector.EndEffectorHelper(self.robot_name, group_name, self.get_control_frame(group_name), self.tf_listener)
                self.end_effector_display[group_name].populate_data(self.get_group_links(group_name), self.get_urdf_model(), self.get_srdf_model())

            return True

        except :
            rospy.logerr(str("PathPlanner::add_planning_group() -- Robot " + self.robot_name + " has problem setting up group: " + group_name))
            return False

   

    def get_ee_parent_group(self, ee) :
        if ee in self.srdf_model.end_effectors :
            return self.srdf_model.end_effectors[ee].group
        return ""

    def get_child_mesh(self, ee_link) :
        mesh_name = ""
        try :
            mesh_name = ee_link.visual.geometry.filename
            return mesh_name
        except :
            try :
                return self.get_child_mesh(self.urdf_model.link_map[ee_link.name].child)
            except :
                return ""

    def print_group_info(self, group_name) :
        if self.has_group(group_name) :
            rospy.loginfo(str("============================================================"))
            rospy.loginfo(str("============ Robot Name: " + self.robot_name))
            rospy.loginfo(str("============ Group: " + group_name))
           
            if group_name in self.get_group_names() :
                rospy.loginfo(str("============ Type: " + str(self.group_types[group_name])))
                rospy.loginfo(str("============ PathPlanner Planning Frame: " + str(self.get_group_planning_frame(group_name))))
                # rospy.loginfo(str("============ PathPlanner Pose Ref Frame: " + self.get_pose_reference_frame(group_name)))
                # rospy.loginfo(str("============ PathPlanner Goal Tolerance: " + self.get_goal_tolerance(group_name)))
                # rospy.loginfo(str("============ PathPlanner Goal Joint Tolerance: " + self.get_goal_joint_tolerance(group_name)))
                # rospy.loginfo(str("============ PathPlanner Goal Position Tolerance: " + self.get_goal_position_tolerance(group_name)))
                # rospy.loginfo(str("============ PathPlanner Goal Orientation Tolerance: " + self.get_goal_orientation_tolerance(group_name)))
                rospy.loginfo(str("============ Control Frame: " + str(self.get_control_frame(group_name))))
                rospy.loginfo(str("============ Control Mesh: " + str(self.get_control_mesh(group_name))))
            rospy.loginfo("============================================================")

    def print_basic_info(self) :
        rospy.loginfo(str("============================================================"))
        rospy.loginfo(str("============ Robot Name: " + self.robot_name))
        rospy.loginfo(str("============ Group Names: "))
        for g in self.get_group_names() :
            rospy.loginfo(str("===============: " + g))
        rospy.loginfo(str("============ Planning frame: " + self.get_robot_planning_frame() ))
        rospy.loginfo(str("============================================================"))
        for g in self.get_group_names() :
            self.print_group_info(g)

    def get_group_type(self, group_name) :
        if self.has_group(group_name) :
            return self.group_types[group_name]
        else :
            return ""

    def get_end_effector_names(self) :
        ee_list = []
        for g in self.group_types.keys() :
            if self.group_types[g] == "endeffector" : ee_list.append(g)
        return ee_list

    def get_control_frame(self, group_name) :
        if self.has_group(group_name) :
            if self.has_end_effector_link(group_name) :
                return self.get_end_effector_link(group_name)
            elif self.group_types[group_name] == "endeffector" :
                if group_name in self.srdf_model.group_end_effectors :
                    return self.srdf_model.group_end_effectors[group_name].parent_link
                else :
                    return "world"
            elif self.srdf_model.has_tip_link(group_name) :
                return self.srdf_model.get_tip_link(group_name)
        else :
            return "world"

    def get_stored_group_state(self, group_name, group_state_name) :
        if group_state_name in self.srdf_model.get_group_state_list(group_name) :
            return self.srdf_model.get_group_state(group_name, group_state_name).to_joint_state_msg()
        else :
            return sensor_msgs.msg.JointState()

    def get_stored_state_list(self, group_name) :
        return self.srdf_model.get_group_state_list(group_name)

    def get_control_mesh(self, group_name) :
        return self.control_meshes[group_name]

    def get_control_mesh_pose_offset(self, group_name) :
        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1
        link_name = self.control_frames[group_name]
        if link_name in self.urdf_model.link_map:
            link = self.urdf_model.link_map[link_name]
            p = link_origin_to_pose(link)
        return p

    def get_control_mesh_scale(self, group_name) :
        s = [1]*3
        link_name = self.control_frames[group_name]
        if link_name in self.urdf_model.link_map:
            link = self.urdf_model.link_map[link_name]
            s = link.visual.geometry.scale
           
        if s == None: s = [1]*3
        return s

    def get_group_links(self, group) :
        return self.srdf_model.get_group_links(group)

    def get_group_joints(self, group) :
        return self.srdf_model.get_group_joints(group)

    def get_urdf_model(self) :
        return self.urdf_model

    def get_srdf_model(self) :
        return self.srdf_model

    def get_trajectory_display_markers(self, group) :
        if group in self.trajectory_display_markers : return self.trajectory_display_markers[group]
        else : return visualization_msgs.msg.MarkerArray()

    def get_base_frame(self, group) :
        if group in self.base_frames : return self.base_frames[group]
        else : return ""

    def set_base_frame(self, group, base_frame) :
        self.base_frames[group] = base_frame

    def set_control_offset(self, group, offset) :
        self.control_offset[group] = offset

    def set_display_mode(self, group, mode) :
        self.display_modes[group] = mode

    def joint_state_callback(self, data):
        self.currentState = data


    # publish path plan to the visualization topic (a MarkerArray you can view in RViz)
    def publish_path_data(self, jt, group) :
        if jt != None :
            # first clear out the last one cause RViz is terrible
            self.clear_published_path(group)
            
            # only do it if it is NOT an end-effector
            if self.group_types[group] != "endeffector" :
                path_visualization_marker_array = self.joint_trajectory_to_marker_array(jt, group, self.display_modes[group])
                self.path_visualization.publish(path_visualization_marker_array)


    # publish a dummy MarkerArray so that RViz clears things out
    def clear_published_path(self,group) :
        markers = visualization_msgs.msg.MarkerArray()
        markers.markers = []
        for m in self.marker_store[group].markers :
            marker = copy.deepcopy(m)
            marker.action = visualization_msgs.msg.Marker.DELETE
            markers.markers.append(marker)
        self.path_visualization.publish(markers)

    def check_valid_plan(self, plan) :
        r = len(plan) > 0 
        if r :
            rospy.loginfo(str("PathPlanner::check_valid_plan() -- Plan Found, size: " + str(len(plan))))
        else :
            rospy.logwarn(str("PathPlanner::check_valid_plan() -- No Plan Found"))
        return r

    # This is the main execution function for sending trajectories to the robot.
    # This will only return True if the plan has been previously (successfully) generated.
    # It will allow the goal to be published using actionlib (which doesnt work for commanding multiple groups at once cause the pything moveit api blocks)
    # or directly to a goal topic (which makes it kind of open loop).
    # If we are using the gripper service to command end-effectors (rather then directly publishing JT msgs), it will do so accordingly here 
    def execute_plan(self, group_name, from_stored=False, wait=True) :
        r = False
        if self.plan_generated[group_name] and self.stored_plans[group_name] :
            rospy.loginfo(str("PathPlanner::execute_plan() -- executing plan for group: " + group_name))
            if self.group_types[group_name] != "endeffector" or not self.gripper_service:
                if self.actionlib :
                    rospy.logdebug("PathPlanner::execute_plan() -- using actionlib")
                    r = self.go(group_name, wait)
                else :
                    rospy.logdebug("PathPlanner::execute_plan() -- publishing to topic")

                    jt = self.translate_trajectory_msg(group_name, self.stored_plans[group_name])
                    
                    N = len(jt.goal.trajectory.points)
                    rospy.logwarn(str("executing path of " + str(N) + " points"))

                    self.command_topics[group_name].publish(jt)
                    r = True # no better way for monitoring success here as it is just an open-loop way of publishing the path
            else :
                r = self.publish_to_gripper_service(group_name, self.stored_plans[group_name])
        else :
            rospy.logerr(str("PathPlanner::execute_plan() -- no plan for group" + group_name + " yet generated."))
            r = False
        rospy.logdebug(str("PathPlanner::execute_plan() -- plan execution: " + str(r)))
        return r


    # There is the option of publishing the JT topic directly to the actionlib goal topic, or to the "bridge" topic
    # that is necessary when commanding a robot using ROS groovy (the JT msg structure changed in hydro -- thanks osrf)  
    def translate_trajectory_msg(self, group_name, jt_in) :
        if group_name in self.bridge_topic_map : 
            rospy.logdebug(str("PathPlanner::translate_trajectory_msg() -- publishing to bridge topic: " + group_name))
            jt = self.translate_to_bridge_msg(jt_in)
        else :
            rospy.logdebug("PathPlanner::translate_trajectory_msg() -- publishing to actionlib goal topic")
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

    # for end-effectors that dont take JointTrajectory inputs (e.g., the PR2), this method will let you bypass this
    # to send a service call to any custom node that will interpret the JT as whatever is necessary
    def publish_to_gripper_service(self, group, traj) :
        try:
            rospy.loginfo("PathPlanner::publish_to_gripper_service() -- calling gripper service")
            resp = self.gripper_service(traj, group, "end_effector_pose") # did i hardcode this? FIXME!! should be something like "Left Hand Close"
            return resp.result
        except rospy.ServiceException, e:
            rospy.logerr("PathPlanner::publish_to_gripper_service() -- gripper service call failed")
            return False


    # convert the JointTractory msg to a MarkerArray msg that can be vizualized in RViz
    def joint_trajectory_to_marker_array(self, joint_trajectory, group, display_mode) :

        markers = visualization_msgs.msg.MarkerArray()
        markers.markers = []
        # joint_start = self.robot.get_current_state().joint_state
        num_points = len(joint_trajectory.points)
        if num_points == 0 : return markers
        idx = 0

        ee_offset = toPose((0,0,0), (0,0,0,1))

        if display_mode == "all_points" :

            for point in joint_trajectory.points[1:num_points:self.path_increment] :
                waypoint_markers, end_pose, last_link = self.create_marker_array_from_joint_array(group, joint_trajectory.joint_names, point.positions, self.get_group_planning_frame(group), idx, self.plan_color[3])
                idx += self.group_id_offset[group]
                idx += len(waypoint_markers)
                for m in waypoint_markers: markers.markers.append(m)

                if self.has_end_effector_link(group) and self.group_types[group] == "manipulator":
                    ee_group = self.srdf_model.end_effectors[self.end_effector_map[group]].group
                    ee_root_frame = self.end_effector_display[ee_group].get_root_frame()
                    if last_link != ee_root_frame :
                        self.tf_listener.waitForTransform(last_link, ee_root_frame, rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.tf_listener.lookupTransform(last_link, ee_root_frame, rospy.Time(0))
                        rot = normalize_vector(rot)
                        ee_offset = toPose(trans, rot)
                        
                    offset_pose = toMsg(end_pose*fromMsg(ee_offset))
                    end_effector_markers = self.end_effector_display[ee_group].get_current_position_marker_array(offset=offset_pose, scale=1, color=self.plan_color, root=self.get_group_planning_frame(group), idx=idx)
                    for m in end_effector_markers.markers: markers.markers.append(m)
                    idx += len(end_effector_markers.markers)

        elif display_mode == "last_point" :

            if num_points > 0 :
                points = joint_trajectory.points[num_points-1]
                waypoint_markers, end_pose, last_link = self.create_marker_array_from_joint_array(group,joint_trajectory.joint_names, points.positions, self.get_group_planning_frame(group), idx, self.plan_color[3])
                for m in waypoint_markers: markers.markers.append(m)
                idx += self.group_id_offset[group]
                idx += len(waypoint_markers)

                if self.has_end_effector_link(group) and self.group_types[group] == "manipulator":
                    ee_group = self.srdf_model.end_effectors[self.end_effector_map[group]].group
                    ee_root_frame = self.end_effector_display[ee_group].get_root_frame()
                    if last_link != ee_root_frame :
                        self.tf_listener.waitForTransform(last_link, ee_root_frame, rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.tf_listener.lookupTransform(last_link, ee_root_frame, rospy.Time(0))
                        rot = normalize_vector(rot)
                        ee_offset = toPose(trans, rot)

                    offset_pose = toMsg(end_pose*fromMsg(ee_offset))
                    end_effector_markers = self.end_effector_display[ee_group].get_current_position_marker_array(offset=offset_pose, scale=1, color=self.plan_color, root=self.get_group_planning_frame(group), idx=idx)
                    for m in end_effector_markers.markers: markers.markers.append(m)
                    idx += len(end_effector_markers.markers)

        self.marker_store[group] = markers
        self.trajectory_display_markers[group] = copy.deepcopy(markers)

        return markers


    def get_joint_chain(self, first_link, last_link) :
        joints = []
        link = last_link
        while link != first_link :
            for j in self.urdf_model.joint_map.keys() :
                if self.urdf_model.joint_map[j].child == link : 
                    joints.append(self.urdf_model.joint_map[j].name)
                    link = self.urdf_model.joint_map[j].parent
                    break
        joints.reverse()
        return joints

    def create_marker_array_from_joint_array(self, group, names, joints, root_frame, idx, alpha) :
        markers = []
        T_acc = kdl.Frame()
        T_kin = kdl.Frame()
        now = rospy.get_rostime()

        first_joint = True

        first_joint_name = names[0]
        last_joint_name = names[len(names)-1]
        joint = first_joint_name
     
        parent_link = self.urdf_model.link_map[self.urdf_model.joint_map[first_joint_name].parent]
        last_link = self.urdf_model.link_map[self.get_control_frame(group)]
        full_names = self.get_joint_chain(parent_link.name, last_link.name)

        for joint in full_names :

            marker = visualization_msgs.msg.Marker()
            parent_link = self.urdf_model.link_map[self.urdf_model.joint_map[joint].parent]
            child_link = self.urdf_model.link_map[self.urdf_model.joint_map[joint].child]
            model_joint = self.urdf_model.joint_map[joint]

            try :
                joint_val = joints[names.index(joint)]
                T_joint = get_joint_rotation(model_joint.axis, joint_val)
            except :
                joint_val = 0.0
                T_joint = kdl.Frame()

            if first_joint :
                first_joint = False
                self.tf_listener.waitForTransform(root_frame, parent_link.name, rospy.Time(0), rospy.Duration(5.0))
                (trans, rot) = self.tf_listener.lookupTransform(root_frame, parent_link.name, rospy.Time(0))
                rot = normalize_vector(rot)
                T_acc = fromMsg(toPose(trans,rot))

            T_kin = fromMsg(joint_origin_to_pose(model_joint))
            T_acc = T_acc*T_kin*T_joint

            if link_has_mesh(child_link) :
                T_viz = fromMsg(link_origin_to_pose(child_link))
                T_link = T_acc*T_viz
                marker.pose = toMsg(T_link)
                marker.header.frame_id = root_frame
                marker.header.stamp = now
                marker.ns = self.robot_name
                marker.text = joint
                marker.id = self.group_id_offset[group] + idx
                try :
                    marker.scale.x = child_link.visual.geometry.scale[0]
                    marker.scale.y = child_link.visual.geometry.scale[1]
                    marker.scale.z = child_link.visual.geometry.scale[2]
                except :
                    marker.scale.x = 1
                    marker.scale.y = 1
                    marker.scale.z = 1
                marker.color.r = self.plan_color[0]
                marker.color.g = self.plan_color[1]
                marker.color.b = self.plan_color[2]
                marker.color.a = self.plan_color[3]
                idx += 1
                marker.mesh_resource = child_link.visual.geometry.filename
                marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE
                marker.action = visualization_msgs.msg.Marker.ADD
                marker.mesh_use_embedded_materials = True
                markers.append(marker)

        return markers, T_acc, child_link.name


    # this is where parse the PathPlanner config package for the robot and figure out what the controller names are 
    def lookup_controller_name(self, group_name) :

        if not group_name in self.group_controllers.keys() :
            import yaml
            try:
                controllers_file = str(RosPack().get_path(self.config_package) + "/config/controllers.yaml") 
                rospy.logdebug("Controller yaml: " + controllers_file)
                controller_config = yaml.load(file(controllers_file, 'r'))
            except :
                rospy.logerr("PathPlanner::lookup_controller_name() -- Error loading controllers.yaml")

            joint_list = self.get_group_joints(group_name)
            
            jn = joint_list[0]
            for j in joint_list:
                if self.urdf_model.joint_map[j].type != "fixed" :
                    jn = j
                    break
            self.group_controllers[group_name] = ""

            for c in controller_config['controller_list'] :
                if jn in c['joints'] :
                    self.group_controllers[group_name] = c['name'] + "/" + c['action_ns']

        rospy.logdebug(str("PathPlanner::lookup_controller_name() -- Found Controller " + self.group_controllers[group_name]  + " for group " + group_name))
        return self.group_controllers[group_name]

    def lookup_bridge_topic_name(self, controller_name) :
        bridge_topic_name = rospy.get_param(str(controller_name + "/bridge_topic"), "")
        return bridge_topic_name

    def tear_down(self) :
        for k in self.end_effector_display.keys() :
            self.end_effector_display[k].stop_offset_update_thread()
        

    def create_plan_to_target(self, group_name, pt) :

        rospy.loginfo(str("PathPlanner::create_plan_to_target() == Robot Name: " + self.robot_name))
        rospy.loginfo(str("PathPlanner::create_plan_to_target() ===== PathPlanner Group Name: " + group_name))
        rospy.loginfo(str("PathPlanner::create_plan_to_target() ===== Generating Plan"))
        
        # transform the goal to the robot/group planning frame 
        if pt.header.frame_id != self.get_group_planning_frame(group_name) :
            self.tf_listener.waitForTransform(pt.header.frame_id, self.get_group_planning_frame(group_name), rospy.Time(0), rospy.Duration(5.0))
            pt = self.tf_listener.transformPose(self.get_group_planning_frame(group_name), pt)
        pt.header.stamp = rospy.Time.now()
        pt.header.seq = int(random.random()*10000000) # needs a unique ID for MoveIt not to get confused (dumb PathPlanner thing)

        # clear flag
        self.plan_generated[group_name] = False
        
        # call the abstract method
        self.stored_plans[group_name] = self.plan_to_cartesian_goal(group_name, pt) 
        
        # check to make sure the plan has a non-0 amount of waypoints
        if self.stored_plans[group_name] :
            self.plan_generated[group_name] = self.check_valid_plan(self.stored_plans[group_name].points)
        
        # if the plan was found publish it to be displayed in RViz as a MarkerArray
        if self.plan_generated[group_name] :
            self.publish_path_data(self.stored_plans[group_name], group_name)

        return self.plan_generated[group_name]


    # computes a PathPlanner JointTrajectory to a single joint goal. 
    def create_joint_plan_to_target(self, group_name, js) :

        rospy.loginfo(str("PathPlanner::create_joint_plan_to_target() == Robot Name: " + self.robot_name))
        rospy.loginfo(str("PathPlanner::create_joint_plan_to_target() ===== PathPlanner Group Name: " + group_name))
        rospy.loginfo(str("PathPlanner::create_joint_plan_to_target() ===== Generating Joint Plan "))

        # set header nonsense
        js.header.stamp = rospy.get_rostime()
        js.header.frame_id = self.get_robot_planning_frame()

        # clear flag
        self.plan_generated[group_name] = False
        
        # call the abstract method
        self.stored_plans[group_name] = self.plan_to_joint_goal(group_name, js) 

        # check to make sure the plan has a non-0 amount of waypoints
        if self.stored_plans[group_name] :
            self.plan_generated[group_name] = self.check_valid_plan(self.stored_plans[group_name].points)

        # if the plan was found publish it to be displayed in RViz as a MarkerArray
        if self.plan_generated[group_name] :
            self.publish_path_data(self.stored_plans[group_name], group_name)

        return self.plan_generated[group_name]


    # computes a PathPlanner JointTrajectory to a random Joint goal in the workspace. 
    def create_random_target(self, group_name) :

        rospy.loginfo(str("PathPlanner::create_random_target() == Robot Name: " + self.robot_name))
        rospy.loginfo(str("PathPlanner::create_random_target() ===== PathPlanner Group Name: " + group_name))
        rospy.loginfo(str("PathPlanner::create_random_target() ===== Generating Random Joint Plan"))
        
        # clear flag
        self.plan_generated[group_name] = False
        
        # call the abstract method
        self.stored_plans[group_name] = self.plan_to_random_goal(group_name) 

        # check to make sure the plan has a non-0 amount of waypoints
        if self.stored_plans[group_name] :
            self.plan_generated[group_name] = self.check_valid_plan(self.stored_plans[group_name].points)

        # if the plan was found publish it to be displayed in RViz as a MarkerArray
        if self.plan_generated[group_name] :
            self.publish_path_data(self.stored_plans[group_name], group_name)

        return self.plan_generated[group_name]

    # This will create a path plan from a list of waypoints.
    # It will also transpose these waypoints to the robot's planning frame. 
    def create_path_plan(self, group_name, frame_id, pt_list) :

        rospy.loginfo(str("PathPlanner::create_path_plan() -- Robot Name: " + self.robot_name))
        rospy.loginfo(str("PathPlanner::create_path_plan() ---- PathPlanner Group Name: " + group_name))
        rospy.loginfo(str("PathPlanner::create_path_plan() ---- Creating Path Plan"))

        waypoints = []

        rospy.logdebug("PathPlanner::create_path_plan() -- transforming input waypoint list")
        for p in pt_list :
            pt = geometry_msgs.msg.PoseStamped()
            pt.pose = p
            rospy.logdebug(str("PathPlanner::create_path_plan() -- INPUT FRAME: " + frame_id))
            rospy.logdebug(str("PathPlanner::create_path_plan() -- PLANNING FRAME: " + self.get_group_planning_frame(group_name)))
            pt.header.frame_id = frame_id
            if pt.header.frame_id != self.get_group_planning_frame(group_name) :
                self.tf_listener.waitForTransform(pt.header.frame_id, self.get_group_planning_frame(group_name), rospy.Time(0), rospy.Duration(5.0))
                pt = self.tf_listener.transformPose(self.get_group_planning_frame(group_name), pt)
            pt.header.frame_id = self.get_group_planning_frame(group_name)
            waypoints.append(copy.deepcopy(pt.pose))
                   
        # if there is more then one waypoint, use the PathPlanner interface to create a Cartesian trajectory through each.
        # this necessary as this is the only way to compute trajectories for multiple points.  FIXME
        if len(waypoints) > 0 :
            rospy.logwarn("PathPlanner::create_path_plan() -- planning as Cartesian path")
            self.stored_plans[group_name] = self.plan_cartesian_path(group_name, waypoints)                
        else :
            rospy.logwarn("PathPlanner::create_path_plan() -- planning as Joint path")
            # if there is just one waypoint, call the create_plan_to_target() to create a joint traj to get there
            pt = geometry_msgs.msg.PoseStamped()
            pt.header.frame_id = self.get_group_planning_frame(group_name)
            pt.header.stamp = rospy.get_rostime()
            pt.pose = waypoints[0]
            self.stored_plans[group_name] = self.plan_to_cartesian_goal(group_name, pt)

        # check to make sure the plan has a non-0 amount of waypoints
        if self.stored_plans[group_name] :
            self.plan_generated[group_name] = self.check_valid_plan(self.stored_plans[group_name].points)

        # if the plan was found publish it to be displayed in RViz as a MarkerArray
        if self.plan_generated[group_name] :
            self.publish_path_data(self.stored_plans[group_name], group_name)

        return self.plan_generated[group_name]

    ############################
    ##### virtual methods ######
    ############################

    ### setup methods
    def setup_group(self, group_name, joint_tolerance, position_tolerance, orientation_tolerance) :
        rospy.logwarn("PathPlanner::setup_group() -- not implemented")
        raise NotImplementedError

    def add_obstacle(self, p, s, n) :
        rospy.logwarn("PathPlanner::add_obstacle() -- collision free path planning not supported")
        raise NotImplementedError

    def get_group_planning_frame(self, group_name) :
        rospy.logerror("PathPlanner::get_group_planning_frame() -- not implemented")
        raise NotImplementedError

    def get_robot_planning_frame(self) :
        rospy.logerror("PathPlanner::get_robot_planning_frame() -- not implemented")
        raise NotImplementedError

    def get_group_joints(self, group_name) :
        rospy.logerror("PathPlanner::get_group_joints() -- not implemented")
        raise NotImplementedError

    def has_end_effector_link(self, group_name) :
        rospy.logerror("PathPlanner::has_end_effector_link() -- not implemented")
        raise NotImplementedError

    def get_end_effector_link(self, group_name) :
        rospy.logerror("PathPlanner::get_end_effector_link() -- not implemented")
        raise NotImplementedError


    def get_goal_tolerance(self, group_name) :
        rospy.logerror("PathPlanner::get_goal_tolerance() -- not implemented")
        raise NotImplementedError

    def get_goal_position_tolerance(self, group_name) :
        rospy.logerror("PathPlanner::get_goal_position_tolerance() -- not implemented")
        raise NotImplementedError

    def get_goal_joint_tolerance(self, group_name) :
        rospy.logerror("PathPlanner::get_goal_joint_tolerance() -- not implemented")
        raise NotImplementedError

    def get_goal_orientation_tolerance(self, group_name) :
        rospy.logerror("PathPlanner::get_goal_orientation_tolerance() -- not implemented")
        raise NotImplementedError

    def set_goal_tolerance(self, group_name) :
        rospy.logerror("PathPlanner::set_goal_tolerance() -- not implemented")
        raise NotImplementedError

    def set_goal_position_tolerance(self, group_name, tol) :
        rospy.logerror("PathPlanner::set_goal_position_tolerance() -- not implemented")
        raise NotImplementedError

    def set_goal_joint_tolerance(self, group_name, tol) :
        rospy.logerror("PathPlanner::set_goal_joint_tolerance() -- not implemented")
        raise NotImplementedError

    def set_goal_orientation_tolerance(self, group_name, tol) :
        rospy.logerror("PathPlanner::set_goal_orientation_tolerance() -- not implemented")
        raise NotImplementedError


    ### planing and execution methods
    def plan_to_cartesian_goal(self, group_name, pt) :
        rospy.logerror("PathPlanner::plan_to_cartesian_point() -- not implemented")
        raise NotImplementedError

    def plan_to_joint_goal(self, group_name, js) :
        rospy.logerror("PathPlanner::plan_to_joint_goal() -- not implemented")
        raise NotImplementedError

    def plan_to_random_goal(self, group_name) :
        rospy.logerror("PathPlanner::plan_to_random_goal() -- not implemented")
        raise NotImplementedError
    
    def plan_cartesian_path(self, group_name, frame_id, pt_list) :
        rospy.logerror("PathPlanner::create_path_plan() -- not implemented")
        raise NotImplementedError

    def go(self, group_name, wait) :
        rospy.logerror("PathPlanner::go() -- not implemented")
        raise NotImplementedError

    def clear_goal_target(self, group_name) :
        rospy.logerror("PathPlanner::clear_goal_target() -- not implemented")
        raise NotImplementedError



    ### multi group functions
    def plan_to_cartesian_goals(self, group_names, pts) :
        rospy.logerror("PathPlanner::plan_to_cartesian_points() -- not implemented")
        raise NotImplementedError

    def plan_to_joint_goals(self, group_names, jss) :
        rospy.logerror("PathPlanner::plan_to_joint_goals() -- not implemented")
        raise NotImplementedError

    def plan_to_random_goals(self, group_names) :
        rospy.logerror("PathPlanner::plan_to_random_goals() -- not implemented")
        raise NotImplementedError
    
    def plan_cartesian_paths(self, group_names, frame_ids, pt_lists) :
        rospy.logerror("PathPlanner::create_path_plans() -- not implemented")
        raise NotImplementedError

    def multigroup_go(self, group_names, wait) :
        rospy.logerror("PathPlanner::multigroup_go() -- not implemented")
        raise NotImplementedError

    def clear_goal_targets(self, group_names) :
        rospy.logerror("PathPlanner::clear_goal_targets() -- not implemented")
        raise NotImplementedError