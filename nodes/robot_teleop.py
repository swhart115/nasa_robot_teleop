#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
import copy
import threading
import tf

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker

import PyKDL as kdl

from nasa_robot_teleop.moveit_interface import *
from nasa_robot_teleop.marker_helper import *
from nasa_robot_teleop.kdl_posemath import *

class RobotTeleop(threading.Thread) :

    def __init__(self, robot_name, manipulator_group_names, joint_group_names):
        super(RobotTeleop,self).__init__()

        self.robot_name = robot_name
        self.manipulator_group_names = manipulator_group_names
        self.joint_group_names = joint_group_names
        self.group_names = manipulator_group_names + joint_group_names
        self.tf_listener = tf.TransformListener()
        self.joint_data = sensor_msgs.msg.JointState()

        self.markers = {}
        self.marker_menus = {}
        self.group_pose_data = {}
        self.control_frames = {}
        self.stored_poses = {}
        self.group_menu_handles = {}
        self.pose_update_thread = {}
        self.pose_store = {}

        self.end_effector_link_data = {}

        # interactive marker server
        self.server = InteractiveMarkerServer(str(self.robot_name + "_teleop"))
        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)

        # set up MoveIt! interface
        self.moveit_interface = MoveItInterface(self.robot_name,str(self.robot_name + "_moveit_config"))
        self.root_frame = self.moveit_interface.get_planning_frame()

        # add user specified goups
        for n in self.manipulator_group_names :
            self.moveit_interface.add_group(n, group_type="manipulator")
        for n in self.joint_group_names :
            self.moveit_interface.add_group(n, group_type="joint")

        # append group list with auto-found end effectors
        for n in self.moveit_interface.get_end_effector_names() :
            self.group_names.append(n)

        # set the control frames for all types of groups
        for n in self.group_names :
            # print "setting control frame: ", self.moveit_interface.get_control_frame(n), " for group: ", n
            self.control_frames[n] = self.moveit_interface.get_control_frame(n)

        # what do we have?
        self.moveit_interface.print_basic_info()

        # set up menu info
        self.menu_options = []
        self.menu_options.append(("Sync To Actual", False))
        self.menu_options.append(("Turn on Joint Control", True))
        self.menu_options.append(("Stored Poses", False))

        # get stored poses from model
        for group in self.group_names :
            self.stored_poses[group] = {}
            for state_name in self.moveit_interface.get_stored_state_list(group) :
                self.stored_poses[group][state_name] = self.moveit_interface.get_stored_group_state(group, state_name)

        # start update threads for manipulators
        for n in self.manipulator_group_names :
            self.start_pose_update_thread(n)

        for n in self.moveit_interface.get_end_effector_names() :
            self.end_effector_link_data[n] = EndEffectorLinkData(self.moveit_interface.get_control_frame(n), self.tf_listener)
            self.end_effector_link_data[n].populate_data(self.moveit_interface.get_group_links(n), self.moveit_interface.get_urdf_model())

        # initialize markers
        self.initialize_group_markers()


    def initialize_group_markers(self) :

        self.group_menu_handles = {}
        self.marker_menus = {}

        for group in self.moveit_interface.groups.keys() :

            self.markers[group] = InteractiveMarker()
            self.markers[group].name = group
            self.marker_menus[group] = MenuHandler()

            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

            if self.moveit_interface.get_group_type(group) == "manipulator" :

                self.markers[group].controls = make6DOFControls()
                self.markers[group].header.frame_id = self.root_frame
                self.markers[group].scale = 0.2

                # insert marker and menus
                self.markers[group].controls.append(menu_control)
                self.server.insert(self.markers[group], self.process_feedback)
                self.reset_group_marker(group)

            elif  self.moveit_interface.get_group_type(group) == "joint" :

                self.markers[group].header.frame_id = self.moveit_interface.get_control_frame(group)
                mesh = self.moveit_interface.get_control_mesh(group)
                pose = self.moveit_interface.get_control_mesh_pose_offset(group)
                marker = makeMesh( self.markers[group] , mesh, pose, sf=1.02, alpha=0.1 )
                menu_control.markers.append( marker )

                # insert marker and menus
                self.markers[group].controls.append(menu_control)
                self.server.insert(self.markers[group], self.process_feedback)

            elif self.moveit_interface.get_group_type(group) == "endeffector" :

                self.markers[group].header.frame_id = self.moveit_interface.srdf_model.group_end_effectors[group].parent_link
                mesh = self.moveit_interface.get_control_mesh(group)
                pose = self.moveit_interface.get_control_mesh_pose_offset(group)
                marker = makeMesh( self.markers[group], mesh, pose, sf=1.02, alpha=0.1 )
                menu_control.markers.append( marker )

                # add other links
                for link in self.end_effector_link_data[group].get_links() :
                    if self.end_effector_link_data[group].get_link_data(link) :
                        (mesh, pose) = self.end_effector_link_data[group].get_link_data(link)
                        marker = makeMesh( self.markers[group], mesh, pose, sf=1.02, alpha=0.1 )
                        marker.text = link
                        menu_control.markers.append( marker )

                # insert marker and menus
                self.markers[group].controls.append(menu_control)
                self.server.insert(self.markers[group], self.process_feedback)

            # Set up stored pose sub menu
            self.setup_stored_pose_menu(group)

            # add menus to server
            self.marker_menus[group].apply( self.server, group )
            self.server.applyChanges()


    def setup_stored_pose_menu(self, group) :
        for m,c in self.menu_options :
            if m == "Stored Poses" :
                sub_menu_handle = self.marker_menus[group].insert(m)
                for p in self.moveit_interface.get_stored_state_list(group) :
                    self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)
            else :
                self.group_menu_handles[(group,m)] = self.marker_menus[group].insert( m, callback=self.process_feedback )
                if c : self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m)], MenuHandler.UNCHECKED )


    def start_pose_update_thread(self, group) :
        self.group_pose_data[group] = geometry_msgs.msg.PoseStamped()
        try :
            self.pose_update_thread[group] = PoseUpdateThread(group, self.root_frame, self.control_frames[group], self.tf_listener, None)
            self.pose_update_thread[group].start()
        except :
            rospy.logerr("RobotTeleop::start_pose_update_thread() -- unable to start group update thread")


    def reset_group_marker(self, group) :
        _marker_valid = False
        while not _marker_valid:
            if self.pose_update_thread[group].is_valid:
                self.group_pose_data[group] = copy.deepcopy(self.pose_update_thread[group].get_pose_data())
                self.server.setPose(self.markers[group].name, self.group_pose_data[group])
                self.server.applyChanges()
                self.server.setPose(self.markers[group].name, self.group_pose_data[group])
                self.server.applyChanges()
                _marker_valid = True
            rospy.sleep(0.1)

    def joint_state_callback(self, data) :
        self.joint_data = data

    def stored_pose_callback(self, feedback) :
        print "pose callback: "
        for p in self.moveit_interface.get_stored_state_list(feedback.marker_name) :
            if self.group_menu_handles[(feedback.marker_name,"Stored Poses",p)] == feedback.menu_entry_id :
                self.moveit_interface.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                r = self.moveit_interface.execute_plan(feedback.marker_name)
                if not r : rospy.logerr(str("RobotTeleop::process_feedback(pose) -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))
                if self.moveit_interface.get_group_type(feedback.marker_name) == "manipulator" :
                    rospy.sleep(3)
                    self.reset_group_marker(feedback.marker_name)

    def process_feedback(self, feedback) :

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            if feedback.marker_name in self.manipulator_group_names :
                self.pose_store[feedback.marker_name] = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.manipulator_group_names :
                pt = geometry_msgs.msg.PoseStamped()
                pt.header = feedback.header
                pt.pose = feedback.pose
                self.moveit_interface.create_plan_to_target(feedback.marker_name, pt)
                r = self.moveit_interface.execute_plan(feedback.marker_name)
                if not r :
                    rospy.logerr(str("RobotTeleop::process_feedback(mouse) -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.group_names :
                handle = feedback.menu_entry_id
                if handle == self.group_menu_handles[(feedback.marker_name,"Sync To Actual")] :
                    self.reset_group_marker(feedback.marker_name)

        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE :
            if feedback.marker_name in self.manipulator_group_names :
                if not feedback.marker_name in self.pose_store: return
                p = toMsg(fromMsg(self.pose_store[feedback.marker_name]).Inverse()*fromMsg(feedback.pose))
                r = (kdl.Rotation.Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)).GetRPY()
                # print "delta p: ", p
                axis_name =  feedback.control_name
                axis_id = self.axis_map(axis_name)
                axis_delta = self.get_axis(axis_name, p, r)

                self.moveit_interface.groups[feedback.marker_name].shift_pose_target(axis_id, axis_delta)
                self.pose_store[feedback.marker_name] = feedback.pose
                # print self.moveit_interface.groups[feedback.marker_name].plan()

    def axis_map(self, n) :
        if n == "move_x": return 0
        elif n == "move_z": return 1
        elif n == "move_y": return 2
        elif n == "rotate_x": return 3
        elif n == "rotate_z": return 4
        elif n == "rotate_y": return 5

    def get_axis(self, n, p, r) :
        d = 0
        if n == "move_x": d=p.position.x
        elif n == "move_z": d=p.position.y
        elif n == "move_y": d=p.position.z
        elif n == "rotate_x": d=r[0]
        elif n == "rotate_z": d=r[1]
        elif n == "rotate_y": d=r[2]
        # print "Axis[", n, "], shift : ", d
        return d

    def run(self) :
        while True :
            try :
                for group in self.moveit_interface.groups.keys():
                    if self.moveit_interface.get_group_type(group) == "endeffector" :

                        for control in self.markers[group].controls :
                            if control.interaction_mode == InteractiveMarkerControl.BUTTON :
                                for link in self.end_effector_link_data[group].get_links() :
                                    if self.end_effector_link_data[group].get_link_data(link) :
                                        (mesh, pose) = self.end_effector_link_data[group].get_link_data(link)
                                        for marker in control.markers :
                                            if marker.text == link :
                                                marker.pose = pose
                                                marker.action = Marker.MODIFY
                        self.server.insert(self.markers[group], self.process_feedback)
                self.server.applyChanges()
            except :
                rospy.logdebug("RobotTeleop::run() -- could not update thread")
            rospy.sleep(1.5)


class PoseUpdateThread(threading.Thread) :
    def __init__(self, name, root_frame, control_frame, tf_listener, offset_pose) :
        super(PoseUpdateThread,self).__init__()
        self.name = name
        self.pose_data = geometry_msgs.msg.PoseStamped()
        self.tf_listener = tf_listener
        self.control_frame = control_frame
        self.root_frame = root_frame
        self.is_valid = False
        self.offset_pose = offset_pose
        if offset_pose != None :
            self.T_offset = fromMsg(self.offset_pose)
        # print "Creating new PoseUpdateThread"
        # print "\tname: ", name
        # print "\tcontrol frame: ", control_frame
        # print "\troot frame: ", root_frame
        # print "\toffset_pose: ", offset_pose


    def run(self) :
        while True :
            try :
                self.tf_listener.waitForTransform(self.control_frame,self.root_frame, rospy.Time(0), rospy.Duration(2.0))
                (trans, rot) = self.tf_listener.lookupTransform(self.root_frame, self.control_frame, rospy.Time(0))
                if self.offset_pose != None :
                    T = fromMsg(toPose(trans, rot))
                    self.pose_data = toMsg(T*self.T_offset)
                    # if self.control_frame == "r2/right_index_distal" :
                    #     print T
                else :
                    self.pose_data = toPose(trans, rot)
                self.is_valid = True
            except :
                rospy.logdebug("PoseUpdateThread::run() -- could not update thread")
            rospy.sleep(0.1)

    def get_pose_data(self) :
        self.is_valid = False
        return self.pose_data

class EndEffectorLinkData :

    def __init__(self, root_frame, tf_listener) :

        self.link_meshes = {}
        self.link_origins = {}
        self.offset_pose_data = {}
        self.offset_update_thread = {}
        self.links = []
        self.root_frame = root_frame
        self.tf_listener = tf_listener

    def add_link(self, link, mesh, origin) :
        self.link_meshes[link] = mesh
        self.link_origins[link] = origin
        self.links.append(link)

    def populate_data(self, links, urdf) :

        for link in links :
            if not link in urdf.link_map :
                print "EndEffectorLinkData::populate_data() -- link: ", link, " not found in URDF model"
                return

            model_link = urdf.link_map[link]

            if model_link :
                if model_link.visual  :
                    if model_link.visual.geometry  :
                        if model_link.visual.geometry.filename  :
                            mesh = model_link.visual.geometry.filename

                            p = geometry_msgs.msg.Pose()
                            q = (kdl.Rotation.RPY(model_link.visual.origin.rpy[0],model_link.visual.origin.rpy[1],model_link.visual.origin.rpy[2])).GetQuaternion()
                            p.position.x = model_link.visual.origin.xyz[0]
                            p.position.y = model_link.visual.origin.xyz[1]
                            p.position.z = model_link.visual.origin.xyz[2]
                            p.orientation.x = q[0]
                            p.orientation.y = q[1]
                            p.orientation.z = q[2]
                            p.orientation.w = q[3]
                            self.add_link(link, mesh, p)
                            # print p

        self.start_offset_update_thread()


    def has_link(self, link) :
        return link in self.links

    def get_links(self) :
        return self.links

    def get_link_data(self, link) :
        if not self.has_link(link) : return False
        if not self.offset_update_thread[link].get_pose_data() : return False
        return (self.link_meshes[link], self.offset_update_thread[link].get_pose_data())

    def start_offset_update_thread(self) :
        for link in self.links :
            self.offset_pose_data[link] = geometry_msgs.msg.PoseStamped()
            try :
                self.offset_update_thread[link] = PoseUpdateThread(link, self.root_frame, link, self.tf_listener, self.link_origins[link])
                self.offset_update_thread[link].start()
            except :
                rospy.logerr("EndEffectorLinkData::start_offset_update_thread() -- unable to start end effector link offset update thread")

    def get_link_offset(self, link) :
        return self.offset_pose_data[link]



if __name__=="__main__":

    rospy.init_node("RobotTeleop")

    robot  = ""
    manipulator_groups = []
    joint_groups = []
    is_valid = True

    try :
        robot = rospy.get_param("~robot")
    except KeyError :
        rospy.logerr("usage:\n$ rosrun nasa_robot_teleop robot_teleop.py _robot:=<robot_name> _manipulator_groups:=[group_name_1, ..., group_name_n] ... optional: _joint_groups=[group_name_1, ..., group_name_n]")
        is_valid = False

    try :
        manipulator_groups = rospy.get_param("~manipulator_groups")
    except KeyError :
        rospy.logerr("usage:\n$ rosrun nasa_robot_teleop robot_teleop.py _robot:=<robot_name> _manipulator_groups:=[group_name_1, ..., group_name_n] ... _joint_groups=[group_name_1, ..., group_name_n]")
        is_valid = False

    joint_groups = rospy.get_param("~joint_groups")

    if is_valid :
        try :
            robot = RobotTeleop(robot, manipulator_groups, joint_groups)
            robot.start()
        except rospy.ROSInterruptException :
            pass

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            r.sleep()
        rospy.spin()


