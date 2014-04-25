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

    def __init__(self, robot_name, group_names):

        self.robot_name = robot_name
        self.group_names = group_names
        self.tf_listener = tf.TransformListener()
        self.joint_data = sensor_msgs.msg.JointState()

        self.server = InteractiveMarkerServer(str(self.robot_name + "_teleop"))
        self.moveit_interface = MoveItInterface(self.robot_name,str(self.robot_name + "_moveit_config"))

        self.markers = {}
        self.marker_menus = {}
        self.group_pose_data = {}
        self.control_frames = {}

        self.root_frame = self.moveit_interface.get_planning_frame()
        for n in self.group_names :
            self.moveit_interface.add_group(n)
            self.control_frames[n] = self.moveit_interface.get_control_frame(n)

        self.group_menu_handles = {}
        self.group_menu_options = []
        self.group_menu_options.append(("Go To Ready Pose", False))
        self.group_menu_options.append(("Edit Control Point", True))
        self.group_menu_options.append(("Reset Control Point", False))
        self.group_menu_options.append(("Sync To Actual", False))

        self.group_ready_pose = {}
        for group in self.group_names :
            self.group_ready_pose[group] = self.moveit_interface.get_stored_group_state(group, str(group + "_ready_pose"))
        self.initialize_group_markers()

        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)


    def initialize_group_markers(self) :

        self.pose_update_thread = {}
        self.group_menu_handles = {}
        self.marker_menus = {}

        for group in self.group_names :

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
            for m,c in self.group_menu_options :
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


    def process_feedback(self, feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.group_names :
                pt = geometry_msgs.msg.PoseStamped()
                pt.header = feedback.header
                pt.pose = feedback.pose
                self.moveit_interface.create_plan_to_target(feedback.marker_name, pt)
                r = self.moveit_interface.execute_plan(feedback.marker_name)
                if not r :
                    rospy.logerr(str("RobotTeleop::process_feedback() -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.group_names :
                handle = feedback.menu_entry_id
                if handle == self.group_menu_handles[(feedback.marker_name,"Go To Ready Pose")] :
                    self.moveit_interface.create_joint_plan_to_target(feedback.marker_name, self.group_ready_pose[feedback.marker_name])
                    r = self.moveit_interface.execute_plan(feedback.marker_name)
                    if not r :
                        rospy.logerr(str("RobotTeleop::process_feedback() -- failed moveit execution for group: " + feedback.marker_name + ". re-synching..."))
                    rospy.sleep(3)
                    self.reset_group_marker(feedback.marker_name)
                if handle == self.group_menu_handles[(feedback.marker_name,"Sync To Actual")] :
                    self.reset_group_marker(feedback.marker_name)

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
    groups = []

    is_valid = True
    try :
        robot = rospy.get_param("~robot")
    except KeyError :
        rospy.logerr("usage:\n$ rosrun nasa_robot_teleop RobotTeleop.py _robot:=<robot_name> _groups:=[group_name_1, ..., group_name_n]")
        is_valid = False

    try :
        groups = rospy.get_param("~groups")
    except KeyError :
        rospy.logerr("usage:\n$ rosrun nasa_robot_teleop RobotTeleop.py _robot:=<robot_name> _groups:=[group_name_1, ..., group_name_n]")
        is_valid = False

    if is_valid :
        try :
            robot = RobotTeleop(robot, groups)
        except rospy.ROSInterruptException :
            pass

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            r.sleep()
        rospy.spin()


