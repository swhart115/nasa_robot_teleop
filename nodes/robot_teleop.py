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

import PyKDL as kdl

from nasa_robot_teleop.moveit_interface import *
from nasa_robot_teleop.marker_helper import *
from nasa_robot_teleop.kdl_posemath import *

class RobotTeleop :

    def __init__(self, robot_name, manipulator_group_names, joint_group_names):

        self.robot_name = robot_name
        self.manipulator_group_names = manipulator_group_names
        self.joint_group_names = joint_group_names
        self.group_names = manipulator_group_names + joint_group_names
        self.tf_listener = tf.TransformListener()
        self.joint_data = sensor_msgs.msg.JointState()

        self.server = InteractiveMarkerServer(str(self.robot_name + "_teleop"))
        self.moveit_interface = MoveItInterface(self.robot_name,str(self.robot_name + "_moveit_config"))

        self.markers = {}
        self.marker_menus = {}
        self.group_pose_data = {}
        self.control_frames = {}
        self.stored_poses = {}

        self.root_frame = self.moveit_interface.get_planning_frame()
        for n in self.manipulator_group_names :
            self.moveit_interface.add_group(n, group_type="manipulator")
            self.control_frames[n] = self.moveit_interface.get_control_frame(n)
        for n in self.joint_group_names :
            self.moveit_interface.add_group(n, group_type="joint")
            self.control_frames[n] = self.moveit_interface.get_control_frame(n)
        for n in self.moveit_interface.get_end_effector_names() :
            self.group_names.append(n)
            self.control_frames[n] = self.moveit_interface.get_control_frame(n)

        self.moveit_interface.print_basic_info()

        self.group_menu_handles = {}
        self.manipulator_group_menu_options = []
        self.manipulator_group_menu_options.append(("Go To Ready Pose", False))
        # self.manipulator_group_menu_options.append(("Edit Control Point", True))
        # self.manipulator_group_menu_options.append(("Reset Control Point", False))
        self.manipulator_group_menu_options.append(("Sync To Actual", False))
        self.manipulator_group_menu_options.append(("Stored Poses", False))

        self.joint_group_menu_options = []
        self.joint_group_menu_options.append(("Go To Ready Pose", False))
        self.joint_group_menu_options.append(("Turn on Joint Control", True))
        self.joint_group_menu_options.append(("Stored Poses", False))

        self.endeffector_group_menu_options = []
        self.endeffector_group_menu_options.append(("Turn on Joint Control", True))
        self.endeffector_group_menu_options.append(("Stored Poses", False))

        self.group_ready_pose = {}
        for group in self.group_names :
            self.group_ready_pose[group] = self.moveit_interface.get_stored_group_state(group, str(group + "_ready_pose"))

        for group in self.group_names :
            self.stored_poses[group] = {}
            print self.moveit_interface.get_stored_state_list(group)
            for state_name in self.moveit_interface.get_stored_state_list(group) :
                print "found group state: ", state_name
                self.stored_poses[group][state_name] = self.moveit_interface.get_stored_group_state(group, state_name)

        self.initialize_group_markers()

        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)


    def initialize_group_markers(self) :

        self.pose_update_thread = {}
        self.group_menu_handles = {}
        self.marker_menus = {}

        for group in self.moveit_interface.groups.keys() :

            if self.moveit_interface.get_group_type(group) == "manipulator" :

                self.markers[group] = InteractiveMarker()
                self.markers[group].controls = make6DOFControls()
                self.markers[group].header.frame_id = self.root_frame
                self.markers[group].name = group
                self.markers[group].scale = 0.2

                menu_control = InteractiveMarkerControl()
                menu_control.interaction_mode = InteractiveMarkerControl.MENU
                # marker = makeMesh( int_marker, mesh_name, p, 1.02 )
                # menu_control.markers.append( marker )
                self.markers[group].controls.append(menu_control)
                self.marker_menus[group] = MenuHandler()
                for m,c in self.manipulator_group_menu_options :
                    if m == "Stored Poses" :
                        sub_menu_handle = self.marker_menus[group].insert(m)
                        for p in self.moveit_interface.get_stored_state_list(group) :
                            self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.pose_callback)
                    else :
                        self.group_menu_handles[(group,m)] = self.marker_menus[group].insert( m, callback=self.process_feedback )
                        if c : self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m)], MenuHandler.UNCHECKED )


                self.group_pose_data[group] = geometry_msgs.msg.PoseStamped()
                try :
                    self.pose_update_thread[group] = PoseUpdateThread(group, self.root_frame, self.control_frames[group], self.tf_listener)
                    self.pose_update_thread[group].start()
                except :
                    rospy.logerr("RobotTeleop::initializeGroupMarkers() -- unable to start group update thread")

                self.server.insert(self.markers[group], self.process_feedback)
                self.reset_group_marker(group)

                self.marker_menus[group].apply( self.server, group )
                self.server.applyChanges()

            elif  self.moveit_interface.get_group_type(group) == "joint" :

                self.markers[group] = InteractiveMarker()
                self.markers[group].header.frame_id = self.moveit_interface.get_control_frame(group)
                self.markers[group].name = group
                # self.markers[group].pose = pose

                mesh = self.moveit_interface.get_control_mesh(group)
                pose = self.moveit_interface.get_control_mesh_pose_offset(group)
                marker = makeMesh( self.markers[group] , mesh, pose, sf=1.02 )

                menu_control = InteractiveMarkerControl()
                menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
                menu_control.markers.append( marker )
                self.markers[group].controls.append(menu_control)

                self.marker_menus[group] = MenuHandler()
                for m,c in self.joint_group_menu_options :
                    if m == "Stored Poses" :
                        sub_menu_handle = self.marker_menus[group].insert(m)
                        for p in self.moveit_interface.get_stored_state_list(group) :
                            print "Adding menu option for pose: ", p
                            self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.pose_callback)
                    else :
                        self.group_menu_handles[(group,m)] = self.marker_menus[group].insert( m, callback=self.process_feedback )
                        if c : self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m)], MenuHandler.UNCHECKED )

                self.server.insert(self.markers[group], self.process_feedback)
                self.marker_menus[group].apply( self.server, group )
                self.server.applyChanges()

            elif self.moveit_interface.get_group_type(group) == "endeffector" :

                self.markers[group] = InteractiveMarker()
                self.markers[group].header.frame_id = self.moveit_interface.srdf_model.group_end_effectors[group].parent_link
                self.markers[group].name = group
                # self.markers[group].pose = pose

                mesh = self.moveit_interface.get_control_mesh(group)
                pose = self.moveit_interface.get_control_mesh_pose_offset(group)
                marker = makeMesh( self.markers[group] , mesh, pose )

                menu_control = InteractiveMarkerControl()
                menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
                menu_control.markers.append( marker )
                self.markers[group].controls.append(menu_control)

                self.marker_menus[group] = MenuHandler()
                for m,c in self.endeffector_group_menu_options :
                    if m == "Stored Poses" :
                        sub_menu_handle = self.marker_menus[group].insert(m)
                        for p in self.moveit_interface.get_stored_state_list(group) :
                            print "Adding menu option for pose: ", p
                            self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.pose_callback)
                    else :
                        self.group_menu_handles[(group,m)] = self.marker_menus[group].insert( m, callback=self.process_feedback )
                        if c : self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m)], MenuHandler.UNCHECKED )

                self.server.insert(self.markers[group], self.process_feedback)
                self.marker_menus[group].apply( self.server, group )
                self.server.applyChanges()


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


    def pose_callback(self, feedback) :
        print "pose callback: "
        for p in self.moveit_interface.get_stored_state_list(feedback.marker_name) :
            if self.group_menu_handles[(feedback.marker_name,"Stored Poses",p)] == feedback.menu_entry_id :
                print "Found pose ", p, " for group ", feedback.marker_name
                print self.stored_poses[feedback.marker_name][p]
                self.moveit_interface.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                r = self.moveit_interface.execute_plan(feedback.marker_name)
                if not r : rospy.logerr(str("RobotTeleop::process_feedback() -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))


    def process_feedback(self, feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.manipulator_group_names :
                pt = geometry_msgs.msg.PoseStamped()
                pt.header = feedback.header
                pt.pose = feedback.pose
                self.moveit_interface.create_plan_to_target(feedback.marker_name, pt)
                r = self.moveit_interface.execute_plan(feedback.marker_name)
                if not r :
                    rospy.logerr(str("RobotTeleop::process_feedback() -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            print "Menu Select: ", feedback.marker_name
            if feedback.marker_name in self.group_names :
                handle = feedback.menu_entry_id
                if handle == self.group_menu_handles[(feedback.marker_name,"Go To Ready Pose")] :
                    self.moveit_interface.create_joint_plan_to_target(feedback.marker_name, self.group_ready_pose[feedback.marker_name])
                    r = self.moveit_interface.execute_plan(feedback.marker_name)
                    if not r : rospy.logerr(str("RobotTeleop::process_feedback() -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))
                    rospy.sleep(3)
                    self.reset_group_marker(feedback.marker_name)
                if handle == self.group_menu_handles[(feedback.marker_name,"Sync To Actual")] :
                    self.reset_group_marker(feedback.marker_name)

                for p in self.moveit_interface.get_stored_state_list(feedback.marker_name) :
                    if (feedback.marker_name,"Stored Poses",p) in self.group_menu_handles :
                        self.moveit_interface.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                        r = self.moveit_interface.execute_plan(feedback.marker_name)
                        if not r : rospy.logerr(str("RobotTeleop::process_feedback() -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))


    def joint_state_callback(self, data) :
        self.joint_data = data


class PoseUpdateThread(threading.Thread) :
    def __init__(self, name, rootFrame, controlFrame, tfListener) :
        super(PoseUpdateThread,self).__init__()
        self.name = name
        self.pose_data = geometry_msgs.msg.PoseStamped()
        self.tf_listener = tfListener
        self.control_frame = controlFrame
        self.root_frame = rootFrame
        self.is_valid = False

    def run(self) :
        while True :
            try :
                self.tf_listener.waitForTransform(self.control_frame,self.root_frame, rospy.Time(0), rospy.Duration(2.0))
                (trans, rot) = self.tf_listener.lookupTransform(self.root_frame, self.control_frame, rospy.Time(0))
                self.pose_data = toPose(trans, rot)
                self.is_valid = True
            except :
                rospy.logdebug("PoseUpdateThread::run() -- could not update thread")
            rospy.sleep(0.1)

    def get_pose_data(self) :
        self.is_valid = False
        return self.pose_data

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
        except rospy.ROSInterruptException :
            pass

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            r.sleep()
        rospy.spin()


