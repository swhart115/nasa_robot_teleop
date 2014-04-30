#! /usr/bin/env python

import sys

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import copy

from rospkg import RosPack

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg

import moveit_commander
import moveit_msgs.msg

import tf
import PyKDL as kdl
from srdf_model import SRDFModel
from kdl_posemath import *

import urdf_parser_py as urdf

class MoveItInterface :

    def __init__(self, robot_name, config_package):

        self.robot_name = robot_name
        self.groups = {}
        self.group_types = {}
        self.control_frames = {}
        self.control_meshes = {}
        self.trajectory_publishers = {}
        self.display_modes = {}
        self.marker_store = visualization_msgs.msg.MarkerArray()

        print "============ Setting up MoveIt! for robot: \'", self.robot_name, "\'"
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.obstacle_markers = visualization_msgs.msg.MarkerArray()
        if not self.create_models(config_package) :
            print "MoveItInterface::init() -- failed creating RDF models"
            return

        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)
        self.obstacle_publisher = rospy.Publisher(str('/' + self.robot_name + '/obstacle_markers'), visualization_msgs.msg.MarkerArray)
        for g in self.robot.get_group_names() :
            self.trajectory_publishers[g] = rospy.Publisher(str('/' + self.robot_name + '/' + g + '/move_group/display_planned_path'), moveit_msgs.msg.DisplayTrajectory)
        self.path_visualization = rospy.Publisher(str('/' + self.robot_name + '/move_group/planned_path_visualization'), visualization_msgs.msg.MarkerArray, latch=False)

        self.tf_listener = tf.TransformListener()


    def create_models(self, config_package) :

        print "============ Creating Robot Model from URDF...."
        self.urdf_model = urdf.Robot.from_parameter_server()
        if self.urdf_model == None : return False

        print "============ Creating Robot Model from SRDF...."
        self.srdf_model = SRDFModel(self.robot_name)

        try :
            srdf_filename = str(RosPack().get_path(config_package) + "/config/" + self.robot_name + ".srdf")
            print "============ SRDF Filename: ", srdf_filename
            if self.srdf_model.parse_from_file(srdf_filename) :
                self.srdf_model.print_model(False)
                print "================================================"
            return True
        except :
            print "MoveItInterface()::create_models() -- error parsing SRDF from file"
            return False


    def add_group(self, group_name, group_type="manipulator", joint_tolerance=0.05, position_tolerance=.02, orientation_tolerance=.05) :

        try :
            self.groups[group_name] = moveit_commander.MoveGroupCommander(group_name)
            self.groups[group_name].set_goal_joint_tolerance(joint_tolerance)
            self.groups[group_name].set_goal_position_tolerance(position_tolerance)
            self.groups[group_name].set_goal_orientation_tolerance(orientation_tolerance)
            self.group_types[group_name] = group_type
            self.control_frames[group_name] = ""
            self.control_meshes[group_name] = ""

            # check to see if the group has an associated end effector, and add it if so
            if self.groups[group_name].has_end_effector_link() :
                self.control_frames[group_name] = self.groups[group_name].get_end_effector_link()
                ee_link = self.urdf_model.link_map[self.groups[group_name].get_end_effector_link()]
                self.control_meshes[group_name] = ee_link.visual.geometry.filename
                for ee in self.srdf_model.end_effectors.keys() :
                    if self.srdf_model.end_effectors[ee].parent_group == group_name :
                        self.add_group(self.srdf_model.end_effectors[ee].group, group_type="endeffector",
                            joint_tolerance=0.05, position_tolerance=0.02, orientation_tolerance=0.05)
            elif self.srdf_model.has_tip_link(group_name) :
                self.control_frames[group_name] = self.srdf_model.get_tip_link(group_name)
                ee_link = self.urdf_model.link_map[self.srdf_model.get_tip_link(group_name)]
                self.control_meshes[group_name] = ee_link.visual.geometry.filename
            elif self.group_types[group_name] == "endeffector" :
                self.control_frames[group_name] = self.srdf_model.group_end_effectors[group_name].parent_link
                ee_link = self.urdf_model.link_map[self.control_frames[group_name]]
                self.control_meshes[group_name] = ee_link.visual.geometry.filename

            return True

        except :
            print "MoveItInterface()::add_group() -- Robot ", self.robot_name, " has no group: ", group_name
            return False

    def has_group(self, group_name) :
        return self.robot.has_group(group_name)

    def print_group_info(self, group_name) :
        if self.has_group(group_name) :
            print "============================================================"
            print "============ Robot Name: %s" % self.robot_name
            print "============ Group: ", group_name

            if group_name in self.groups.keys() :
                print "============ Type: ", self.group_types[group_name]
                print "============ MoveIt! Planning Frame: ", self.groups[group_name].get_planning_frame()
                print "============ MoveIt! Pose Ref Frame: ", self.groups[group_name].get_pose_reference_frame()
                print "============ MoveIt! Goal Tolerance: ", self.groups[group_name].get_goal_tolerance()
                print "============ MoveIt! Goal Joint Tolerance: ", self.groups[group_name].get_goal_joint_tolerance()
                print "============ MoveIt! Goal Position Tolerance: ", self.groups[group_name].get_goal_position_tolerance()
                print "============ MoveIt! Goal Orientation Tolerance: ", self.groups[group_name].get_goal_orientation_tolerance()
                print "============ Control Frame: ", self.get_control_frame(group_name)
                print "============ Control Mesh: ", self.get_control_mesh(group_name)

    def print_basic_info(self) :
        print "============================================================"
        print "============ Robot Name: %s" % self.robot_name
        print "============ Group Names: ", self.robot.get_group_names()
        print "============ Planning frame: %s" % self.robot.get_planning_frame()
        print "============================================================"
        for g in self.robot.get_group_names() :
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
            if self.groups[group_name].has_end_effector_link() :
                return self.groups[group_name].get_end_effector_link()
            elif self.group_types[group_name] == "endeffector" :
                if group_name in self.srdf_model.group_end_effectors :
                    return self.srdf_model.group_end_effectors[group_name].parent_link
                else :
                    return "world"
            elif self.srdf_model.has_tip_link(group_name) :
                return self.srdf_model.get_tip_link(group_name)
        else :
            return "world"

    def get_planning_frame(self) :
        return self.robot.get_planning_frame()

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
            p = self.link_origin_to_pose(link)
        return p

    def get_group_links(self, group) :
        return self.srdf_model.get_group_links(group)

    def get_group_joints(self, group) :
        return self.srdf_model.get_group_joints(group)

    def get_urdf_model(self) :
        return self.urdf_model

    def get_srdf_model(self) :
        return self.srdf_model

    def set_display_mode(self, group, mode) :
        self.display_modes[group] = mode

    def joint_state_callback(self, data):
        self.currentState = data

    def clear_published_path(self) :
        markers = visualization_msgs.msg.MarkerArray()
        markers.markers = []
        for m in self.marker_store.markers :
            marker = copy.deepcopy(m)
            marker.action = visualization_msgs.msg.Marker.DELETE
            markers.markers.append(marker)
        self.path_visualization.publish(markers)

    def publish_path_data(self, plan, group) :
        if plan != None :
            self.clear_published_path()
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self.trajectory_publishers[group].publish(display_trajectory)
            path_visualization_marker_array = self.joint_trajectory_to_marker_array(plan, group, self.display_modes[group])
            self.path_visualization.publish(path_visualization_marker_array)

    def create_joint_plan_to_target(self, group_name, js) :
        print "== Robot Name: %s" % self.robot_name
        print "===== MoveIt! Group Name: ", group_name
        print "===== Generating Joint Plan"
        self.groups[group_name].set_joint_value_target(js)
        plan = self.groups[group_name].plan()
        self.publish_path_data(plan, group_name)

    def create_plan_to_target(self, group_name, pt) :
        if pt.header.frame_id != self.groups[group_name].get_planning_frame() :
            self.tf_listener.waitForTransform(pt.header.frame_id, self.groups[group_name].get_planning_frame(), rospy.Time(0), rospy.Duration(5.0))
            pt = self.tf_listener.transformPose(self.groups[group_name].get_planning_frame(), pt)
        print "== Robot Name: %s" % self.robot_name
        print "===== MoveIt! Group Name: %s" % group_name
        print "===== Generating Plan"
        self.groups[group_name].set_pose_target(pt)
        plan = self.groups[group_name].plan()
        self.publish_path_data(plan, group_name)

    def create_random_target(self, group_name) :
        print "== Robot Name: %s" % self.robot_name
        print "===== MoveIt! Group Name: %s" % group_name
        print "===== Generating Random Joint Plan"
        self.groups[group_name].set_random_target()
        plan = self.groups[group_name].plan()
        self.publish_path_data(plan, group_name)

    def execute_plan(self, group_name) :
        r = self.groups[group_name].go(False)
        print "====== Plan Execution: %s" % r
        return r

    def add_collision_object(self, p, s, n) :
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
        self.obstacle_publisher.publish(self.obstacle_markers)

    def joint_trajectory_to_marker_array(self, plan, group, display_mode) :
        markers = visualization_msgs.msg.MarkerArray()
        markers.markers = []
        joint_start = self.robot.get_current_state().joint_state
        num_points = len(plan.joint_trajectory.points)
        idx = 0

        pt_id = 0.0

        if display_mode == "all_points" :
            for point in plan.joint_trajectory.points :
                a = 1-((pt_id/float(num_points))*0.8)
                waypoint_markers = self.create_marker_array_from_joint_array(plan.joint_trajectory.joint_names, point.positions, idx, a)
                idx += len(waypoint_markers)
                for m in waypoint_markers: markers.markers.append(m)
                pt_id += 1
        elif display_mode == "last_point" :
            points = plan.joint_trajectory.points[num_points-1]
            waypoint_markers = self.create_marker_array_from_joint_array(plan.joint_trajectory.joint_names, points.positions, idx, 1)
            for m in waypoint_markers: markers.markers.append(m)

        self.marker_store = markers
        return markers

    def create_marker_array_from_joint_array(self, names, joints, idx, alpha) :
        markers = []
        T_acc = kdl.Frame()
        root_frame = ""
        now = rospy.get_rostime()

        for joint in names :

            marker = visualization_msgs.msg.Marker()
            parent_link = self.urdf_model.link_map[self.urdf_model.joint_map[joint].parent]
            child_link = self.urdf_model.link_map[self.urdf_model.joint_map[joint].child]

            if root_frame == "" : root_frame = parent_link.name
            model_joint = self.urdf_model.joint_map[joint]
            T_kin = fromMsg(self.joint_origin_to_pose(model_joint))

            joint_val = joints[names.index(joint)]
            T_joint = self.get_z_rotation_frame(joint_val)
            T_acc = T_acc*T_kin*T_joint

            if self.link_has_mesh(child_link) :
                T_viz = fromMsg(self.link_origin_to_pose(child_link))
                T_link = T_acc*T_viz
                marker.pose = toMsg(T_link)
                marker.header.frame_id = root_frame
                marker.header.stamp = now
                marker.ns = self.robot_name
                marker.mesh_use_embedded_materials = True
                marker.frame_locked = False
                marker.text = joint
                marker.id = idx
                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 1
                marker.color.a = alpha
                marker.color.r = 0.5
                marker.color.g = 0
                marker.color.b = 1
                idx += 1
                marker.mesh_resource = child_link.visual.geometry.filename
                marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE
                marker.action = visualization_msgs.msg.Marker.ADD
                markers.append(marker)

        return markers

    def get_z_rotation_frame(self, theta) :
        T = kdl.Frame()
        T.M.DoRotZ(theta)
        return T

    def link_has_mesh(self, link) :
        if link.visual :
            if link.visual.geometry :
                if isinstance(link.visual.geometry, urdf.Mesh) :
                    if link.visual.geometry.filename :
                        return True
        else :
            return False

    def link_has_origin(self, link) :
        if link.visual :
            if link.visual.origin :
                return True
        else :
            return False

    def link_origin_to_pose(self, link) :
        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1
        if self.link_has_origin(link) :
            if link.visual.origin.xyz :
                p.position.x = link.visual.origin.xyz[0]
                p.position.y = link.visual.origin.xyz[1]
                p.position.z = link.visual.origin.xyz[2]
            if link.visual.origin.rpy :
                q = (kdl.Rotation.RPY(link.visual.origin.rpy[0],link.visual.origin.rpy[1],link.visual.origin.rpy[2])).GetQuaternion()
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
        return p

    def joint_origin_to_pose(self, joint) :
        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1
        if joint.origin :
            if joint.origin.xyz :
                p.position.x = joint.origin.xyz[0]
                p.position.y = joint.origin.xyz[1]
                p.position.z = joint.origin.xyz[2]
            if joint.origin.rpy :
                q = (kdl.Rotation.RPY(joint.origin.rpy[0],joint.origin.rpy[1],joint.origin.rpy[2])).GetQuaternion()
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
        return p

if __name__ == '__main__':

    rospy.init_node('moveit_intefrace_test')

    moveit_commander.roscpp_initialize(sys.argv)

    try:
        moveit_test = MoveItInterface("r2", "r2_moveit_config")
        moveit_test.add_group("right_arm")
        moveit_test.add_group("left_arm")
        moveit_test.add_group("head", group_type="joint")

        q = (kdl.Rotation.RPY(-1.57,0,0)).GetQuaternion()
        pt = geometry_msgs.msg.PoseStamped()
        pt.header.frame_id = "world"
        pt.header.seq = 0
        pt.header.stamp = rospy.Time.now()
        pt.pose.position.x = -0.3
        pt.pose.position.y = -0.5
        pt.pose.position.z = 1.2
        pt.pose.orientation.x = q[0]
        pt.pose.orientation.y = q[1]
        pt.pose.orientation.z = q[2]
        pt.pose.orientation.w = q[3]
        moveit_test.create_plan_to_target(pt, "left_arm")

        q = (kdl.Rotation.RPY(1.57,0,-1.57)).GetQuaternion()
        pt = geometry_msgs.msg.PoseStamped()
        pt.header.frame_id = "world"
        pt.header.seq = 0
        pt.header.stamp = rospy.Time.now()
        pt.pose.position.x = 0.3
        pt.pose.position.y = -0.5
        pt.pose.position.z = 1.2
        pt.pose.orientation.x = q[0]
        pt.pose.orientation.y = q[1]
        pt.pose.orientation.z = q[2]
        pt.pose.orientation.w = q[3]
        moveit_test.create_plan_to_target(pt, "right_arm")

        r1 = moveit_test.execute_plan("left_arm")
        r2 = moveit_test.execute_plan("right_arm")
        if not r1 : rospy.logerr("moveit_test(left_arm) -- couldn't execute plan")
        if not r2 : rospy.logerr("moveit_test(right_arm) -- couldn't execute plan")

        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass




