#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
import copy
import threading
import tf
import argparse
import PyKDL as kdl

# ros messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker

# helper files
from nasa_robot_teleop.marker_helper import *
from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.pose_update_thread import *
from nasa_robot_teleop.end_effector_helper import *
from nasa_robot_teleop.tolerances import *

# path planner instances
from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.moveit_path_planner import *
from nasa_robot_teleop.atlas_path_planner import *
from nasa_robot_teleop.group_config_parser import *
from nasa_robot_teleop.group_config import *

from navigation_waypoint_control import *

class InteractiveControl:

    def __init__(self, robot_name, planner_type, navigation_frame, group_config_file, planner_config_file):

        self.robot_name = robot_name
        self.group_config_file = group_config_file
        self.planner_config_file = planner_config_file

        self.group_map = []
        
        self.navigation_frame = navigation_frame

        self.tf_listener = tf.TransformListener()
        self.joint_data = sensor_msgs.msg.JointState()
        self.joint_group_map = {}
        
        self.markers = {}
        self.marker_menus = {}
        self.group_menu_handles = {}

        self.posture_markers = {}
        self.posture_markers_on = {}

        self.group_pose_data = {}
        self.control_frames = {}

        self.stored_poses = {}

        self.end_effector_link_data = {}
        self.pose_store = {}
        self.pose_update_thread = {}

        self.auto_plan = {}
        self.auto_execute = {}

        self.position_tolerance_modes = {}
        self.orientation_tolerance_modes = {}
        self.group_position_tolerance_mode = {}
        self.group_orientation_tolerance_mode = {}

        self.gripper_service = None
        self.config_parser = None
        
        # interactive marker server
        self.server = InteractiveMarkerServer(str(self.robot_name + "_teleop"))

        # nav control markers
        self.navigation_controls = NavigationWaypointControl(self.robot_name, self.server, self.navigation_frame)

        # joint state sub
        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)

        # service helpers
        self.add_group_srv = rospy.Service('/interactive_control/add_group', AddGroup, self.handle_add_group)
        self.remove_group_srv = rospy.Service('/interactive_control/remove_group', RemoveGroup, self.handle_remove_group)

        # set up menu info
        self.menu_options = []
        self.menu_options.append(("Plan", False))
        self.menu_options.append(("Execute", False))
        self.menu_options.append(("Plan On Move", True))
        self.menu_options.append(("Execute On Plan", True))
        self.menu_options.append(("Show Path", True))       
        self.menu_options.append(("Toggle Joint Control", False))
        self.menu_options.append(("Sync To Actual", False))
        self.menu_options.append(("Stored Poses", False))       

        # get the groups from the config file
        self.group_map = self.parse_config_file(self.group_config_file)

        # planner instantiatation
        if planner_type == "moveit" :
            self.path_planner = MoveItPathPlanner(self.robot_name, self.planner_config_file)
        elif planner_type == "atlas" :
            self.path_planner = AtlasPathPlanner(self.robot_name, self.planner_config_file)
        else :
            rospy.logerr("InteractiveControl() unrecognized planner type!!")
            exit()

        self.root_frame = self.path_planner.get_robot_planning_frame()
        # set the control frames for all types of groups
        for n in self.get_groups() :
            self.control_frames[n] = self.path_planner.get_control_frame(n)

        self.use_tolerances = self.path_planner.uses_tolerances()

        # load the urdf
        self.urdf = self.path_planner.get_urdf_model()

        # setup the groups
        self.setup_groups()       
        
        # what do we have?  
        self.path_planner.print_basic_info()
       
        # get stored poses from model
        for group in self.get_groups() :
            self.stored_poses[group] = {}
            for state_name in self.path_planner.get_stored_state_list(group) :
                self.stored_poses[group][state_name] = self.path_planner.get_stored_group_state(group, state_name)
       

        # Create EndEffectorHelper objects to help with EE displays
        for n in self.path_planner.get_end_effector_names() :
            self.end_effector_link_data[n] = EndEffectorHelper(self.robot_name, n, self.path_planner.get_control_frame(n), self.tf_listener)
            self.end_effector_link_data[n].populate_data(self.path_planner.get_group_links(n), self.urdf, self.path_planner.get_srdf_model())

        
    def parse_config_file(self, config_file) :
        self.config_parser = GroupConfigParser(config_file)
        return self.config_parser.get_group_map()

    def get_groups(self, group_type=None) :
        if group_type==None :
            l = []
            for t in self.group_map.keys() :
                if self.group_map[t] :
                    l = l+self.group_map[t]
            g = list(set(l))
            return g
        else :
            if not group_type in self.group_map.keys() :
                return []
            return self.group_map[group_type]

    def get_group_type(self, group) :
        for t in self.group_map.keys() :
            if group in self.group_map[t] :
                return t
        return ""

    def setup_groups(self) :
        for g in self.get_groups() :
            self.setup_group(g)

    def setup_group(self, group) :       
        group_type = self.get_group_type(group)
        if not self.path_planner.add_planning_group(group, group_type) :
            self.group_map[group_type].remove(group)
            rospy.logerr(str("InteractiveControl::setup_group() -- planner rejected group: " + group + " of type: " + group_type))
            return False        
        # create interactive markers for group
        return self.initialize_group_markers(group)


    def initialize_all_group_markers(self) :
        self.group_menu_handles = {}
        self.marker_menus = {}
        for g in self.get_groups() :
            self.initialize_group_markers(g)
    
    def initialize_group_markers(self, group) :

        self.auto_execute[group] = False
        self.auto_plan[group] = False
        self.markers[group] = InteractiveMarker()
        self.markers[group].name = group
        self.markers[group].description = group
        self.marker_menus[group] = MenuHandler()
        
        group_type = self.get_group_type(group)

        if group_type == "cartesian" :
            self.initialize_cartesian_group(group)
        elif group_type == "joint" :
            self.initialize_joint_group(group)
        elif group_type == "endeffector" :
            self.initialize_endeffector_group(group)

        # insert marker and menus
        self.server.insert(self.markers[group], self.process_feedback)
        self.auto_execute[group] = True

        # posture stuff
        self.posture_markers[group] = {}
        self.posture_markers_on[group] = False
        
        # Set up stored pose sub menu
        self.setup_sub_menus(group)

        self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Show Path")], MenuHandler.CHECKED )
        self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Plan On Move")], MenuHandler.UNCHECKED )
        
        # add menus to server
        self.marker_menus[group].apply( self.server, group )
        
        self.server.applyChanges()


    def initialize_cartesian_group(self, group) : 
    
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

        if self.path_planner.has_joint_mask(group) :
            if ("Joint Mask", False) not in  self.menu_options :
                self.menu_options.append(("Joint Mask", False))

        if self.use_tolerances :
            if ("Position Tolerance", False) not in  self.menu_options :
                self.menu_options.append(("Position Tolerance", False))
            if ("Angle Tolerance", False) not in  self.menu_options :
                self.menu_options.append(("Angle Tolerance", False))

        self.markers[group].controls = make6DOFControls()
        self.markers[group].header.frame_id = self.root_frame
        self.markers[group].scale = 0.2

        # insert marker and menus
        self.markers[group].controls.append(menu_control)
        self.server.insert(self.markers[group], self.process_feedback)
        self.markers[group].controls.append(menu_control)

        if self.use_tolerances :
            self.position_tolerance_modes[group] = self.path_planner.tolerances.get_tolerance_modes('PositionTolerance')
            self.orientation_tolerance_modes[group] = self.path_planner.tolerances.get_tolerance_modes('OrientationTolerance')
            self.path_planner.set_goal_position_tolerance_mode(group, None)
            self.path_planner.set_goal_orientation_tolerance_mode(group, None)

        # start update threads for cartesian groups
        self.start_pose_update_thread(group)
        self.reset_group_marker(group)
        
 
    def initialize_joint_group(self, group) :    

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

        self.markers[group].header.frame_id = self.path_planner.get_control_frame(group)
        control_frame = self.path_planner.get_control_frame(group)
        jg_links = self.urdf.get_all_child_links(control_frame)
        idx = 0
        for jg_link in jg_links :
            try :
                marker = get_mesh_marker_for_link(jg_link, self.urdf)
                if marker != None :
                    marker.color.r = 1
                    marker.color.g = 1
                    marker.color.b = 1
                    marker.color.a = 0.1
                    marker.scale.x = marker.scale.x*1.03
                    marker.scale.y = marker.scale.y*1.03
                    marker.scale.z = marker.scale.z*1.03
                    marker.id = idx
                    menu_control.markers.append(marker)
                    idx += 1
            except :
                pass

        # insert marker and menus
        self.markers[group].controls.append(menu_control)
        self.server.insert(self.markers[group], self.process_feedback)
        self.markers[group].controls.append(menu_control)
 

    def initialize_endeffector_group(self, group) : 

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

        control_frame = self.path_planner.get_control_frame(group)
        ee_links = self.urdf.get_all_child_links(control_frame)
        self.markers[group].header.frame_id = self.path_planner.srdf_model.group_end_effectors[group].parent_link

        idx = 0
        for ee_link in ee_links :
            try :
                end_effector_marker = get_mesh_marker_for_link(ee_link, self.urdf)

                if end_effector_marker != None :
                    end_effector_marker.color.r = 1
                    end_effector_marker.color.g = 1
                    end_effector_marker.color.b = 1
                    end_effector_marker.color.a = 0.1
                    end_effector_marker.id = idx
                    menu_control.markers.append(end_effector_marker)
                    idx += 1
            except :
                pass

        if self.path_planner.get_group_type(group) == "endeffector" :
            self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Execute On Plan")], MenuHandler.CHECKED )

        self.markers[group].controls.append(menu_control)
 

    def remove_group_markers(self, group) :
        self.server.erase(self.markers[group].name)
        del self.markers[group]
        del self.marker_menus[group]
        self.server.applyChanges()

    def handle_add_group(self, req) :
        resp = AddGroupResponse()
        g = GroupConfig()
        g.name = req.name
        g.type = req.type
        g.joint_list = req.joint_list
        g.root_frame = req.root_frame
        g.control_frame = req.control_frame
        self.setup_group(g)
        resp.id = 0
        return resp

    def handle_remove_group(self, req) :
        resp = RemoveGroupResponse()
        self.remove_group_markers(req.name)
        resp.success = True
        return resp

    def set_gripper_service(self, srv) :
        self.gripper_service = srv
        self.path_planner.set_gripper_service(srv)

    def clear_gripper_service(self) :
        self.gripper_service = None
        self.path_planner.clear_gripper_service()

    def toggle_posture_control(self, group) :

        if self.path_planner.has_joint_map(group):
            joint_map = self.path_planner.get_joint_map(group)

            for idx in range(len(joint_map.names)) :

                jnt = joint_map.names[idx]
                lnk = get_joint_child_link(jnt,self.urdf)
                self.joint_group_map[jnt] = group

                if self.posture_markers_on[group] :
                    self.server.erase(self.posture_markers[group][lnk].name)
                else :
                    joint = self.urdf.joint_map[jnt]
                    int_marker = InteractiveMarker()
                    int_marker.header.frame_id = lnk
                    int_marker.scale = 0.2
                    int_marker.name = jnt
                    int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z = 0,0,0# joint.origin.xyz
                    int_marker.pose.orientation.w = 1
                    int_marker.pose.orientation.x, int_marker.pose.orientation.y, int_marker.pose.orientation.z = 0,0,0# joint.origin.rpy
                    control = InteractiveMarkerControl()
                    control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = axis_to_q(joint.axis)
                    control.name = "rotate"
                    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
                    int_marker.controls.append(control)
                    self.posture_markers[group][lnk] = int_marker
                    self.server.insert(self.posture_markers[group][lnk], self.posture_feedback)

        self.server.applyChanges()
        self.posture_markers_on[group] = not self.posture_markers_on[group] 


    def setup_sub_menus(self, group) :
        for m,c in self.menu_options :
            if m == "Stored Poses" :
                sub_menu_handle = self.marker_menus[group].insert(m)
                for p in self.path_planner.get_stored_state_list(group) :
                    self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)
            elif m == "Position Tolerance" :
                if self.get_group_type(group) == "cartesian" :
                    sub_menu_handle = self.marker_menus[group].insert(m)
                    for p in self.position_tolerance_modes[group] :
                        self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.position_tolerance_callback)
                        self.marker_menus[group].setCheckState(self.group_menu_handles[(group,m,p)], MenuHandler.UNCHECKED )
            elif m == "Angle Tolerance" :
                if self.get_group_type(group) == "cartesian" :
                    sub_menu_handle = self.marker_menus[group].insert(m)
                    for p in self.orientation_tolerance_modes[group] :
                        self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.orientation_tolerance_callback)   
                        self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m,p)], MenuHandler.UNCHECKED )
            elif m == "Joint Mask" :
                sub_menu_handle = self.marker_menus[group].insert(m) 
                if self.path_planner.has_joint_mask(group) and self.path_planner.has_joint_map(group):
                    joint_map = self.path_planner.get_joint_map(group)
                    joint_mask = self.path_planner.get_joint_mask(group)
                    for idx in range(len(joint_map.names)) :
                        jnt = joint_map.names[idx]
                        jnt_id = joint_map.ids[idx]
                        mask_val = joint_mask[idx]
                        self.group_menu_handles[(group,m,jnt)] = self.marker_menus[group].insert(jnt,parent=sub_menu_handle,callback=self.joint_mask_callback)   
                        if mask_val :
                            self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m,jnt)], MenuHandler.CHECKED )        
                        else :
                            self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m,jnt)], MenuHandler.UNCHECKED )        
            else :
                self.group_menu_handles[(group,m)] = self.marker_menus[group].insert( m, callback=self.process_feedback )
                if c : self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m)], MenuHandler.UNCHECKED )


    def start_pose_update_thread(self, group) :
        self.group_pose_data[group] = geometry_msgs.msg.PoseStamped()
        try :
            rospy.logwarn(str("starting pose_update_thread() for group: " + group + " with frame: " + self.control_frames[group]))
            self.pose_update_thread[group] = PoseUpdateThread(group, self.root_frame, self.control_frames[group], self.tf_listener, None)
            self.pose_update_thread[group].start()
        except :
            rospy.logerr("InteractiveControl::start_pose_update_thread() -- unable to start group update thread")


    def reset_group_marker(self, group) :
        _marker_valid = False
        while not _marker_valid:
            if self.pose_update_thread[group].is_valid:
                self.group_pose_data[group] = copy.deepcopy(self.pose_update_thread[group].get_pose_data())
                self.server.setPose(self.markers[group].name, self.group_pose_data[group])
                self.server.applyChanges()
                # What?! do it again? Why? Huh?!
                self.server.setPose(self.markers[group].name, self.group_pose_data[group])
                self.server.applyChanges()
                _marker_valid = True
            rospy.sleep(0.1)


    def joint_state_callback(self, data) :
        self.joint_data = data

    def position_tolerance_callback(self,feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.get_groups() :
                self.group_position_tolerance_mode[feedback.marker_name] = "FULL"
                self.path_planner.set_goal_position_tolerance_mode(feedback.marker_name, "FULL")
                handle = feedback.menu_entry_id
                for p in self.position_tolerance_modes[feedback.marker_name] :
                    if handle == self.group_menu_handles[(feedback.marker_name,"Position Tolerance", p)] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        elif state == MenuHandler.UNCHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            self.group_position_tolerance_mode[feedback.marker_name] = p
                            self.path_planner.set_goal_position_tolerance_mode(feedback.marker_name, p)
                    else :
                        h = self.group_menu_handles[(feedback.marker_name,"Position Tolerance", p)]
                        self.marker_menus[feedback.marker_name].setCheckState( h, MenuHandler.UNCHECKED )
        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()

    def orientation_tolerance_callback(self,feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.get_groups() :
                handle = feedback.menu_entry_id
                self.group_orientation_tolerance_mode[feedback.marker_name] = "FULL"
                self.path_planner.set_goal_orientation_tolerance_mode(feedback.marker_name, "FULL")
                for p in self.orientation_tolerance_modes[feedback.marker_name] :
                    if handle == self.group_menu_handles[(feedback.marker_name,"Angle Tolerance", p)] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        elif state == MenuHandler.UNCHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            self.group_orientation_tolerance_mode[feedback.marker_name] = p
                            self.path_planner.set_goal_orientation_tolerance_mode(feedback.marker_name, p)
                    else :
                        h = self.group_menu_handles[(feedback.marker_name,"Angle Tolerance", p)]
                        self.marker_menus[feedback.marker_name].setCheckState( h, MenuHandler.UNCHECKED )
        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()
                                 
    def joint_mask_callback(self, feedback) :
        if self.path_planner.has_joint_mask(feedback.marker_name) and self.path_planner.has_joint_map(feedback.marker_name) :
            joint_map = self.path_planner.get_joint_map(feedback.marker_name)
            joint_mask = self.path_planner.get_joint_mask(feedback.marker_name)
            if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
                if feedback.marker_name in self.get_groups() :
                    handle = feedback.menu_entry_id
                    for idx in range(len(joint_map.names)) :
                        jnt = joint_map.names[idx]
                        jnt_id = joint_map.ids[idx]
                        if handle == self.group_menu_handles[(feedback.marker_name,"Joint Mask", jnt)] :
                            state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                            if state == MenuHandler.CHECKED:
                                self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                                joint_mask[jnt_id] = False
                                self.path_planner.set_joint_mask(feedback.marker_name, joint_mask)
                            else:
                                self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )                                                      
                                joint_mask[jnt_id] = True
                                self.path_planner.set_joint_mask(feedback.marker_name, joint_mask)
            self.marker_menus[feedback.marker_name].reApply( self.server )
            self.server.applyChanges()

    def stored_pose_callback(self, feedback) :
        for p in self.path_planner.get_stored_state_list(feedback.marker_name) :
            if self.group_menu_handles[(feedback.marker_name,"Stored Poses",p)] == feedback.menu_entry_id :
                if self.auto_execute[feedback.marker_name] :
                    self.path_planner.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                    r = self.path_planner.execute(feedback.marker_name)
                    # r = self.path_planner.plan_joint_goal_and_execute(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                    if not r : rospy.logerr(str("InteractiveControl::process_feedback(pose) -- failed moveitplanner execution for group: " + feedback.marker_name + ". re-synching..."))
                else :
                    self.path_planner.clear_goal_target(feedback.marker_name)
                    self.path_planner.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                if self.path_planner.get_group_type(feedback.marker_name) == "cartesian" :
                    rospy.sleep(3)
                    self.reset_group_marker(feedback.marker_name)

    def posture_feedback(self, feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            js = JointState()
            js.name.append(feedback.marker_name)
            q = feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w
            print rpy
            print theta
            js.position.append(theta)
            group = self.joint_group_map[feedback.marker_name]
            if self.auto_execute[group] :
                self.path_planner.create_joint_plan_to_target(group, js)
                r = self.path_planner.execute(group)
                if not r : rospy.logerr(str("InteractiveControl::posture_feedback(pose) -- failed moveitplanner execution for group: " + feedback.marker_name))
            else :
                self.path_planner.clear_goal_target(group)
                self.path_planner.create_joint_plan_to_target(group, js)
        
    def process_feedback(self, feedback) :
        
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            if feedback.marker_name in self.get_groups('cartesian') :
                self.pose_store[feedback.marker_name] = feedback.pose
        
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.get_groups('cartesian') :
                if self.auto_plan[feedback.marker_name] :
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = feedback.header
                    pt.pose = feedback.pose
                    if self.auto_execute[feedback.marker_name] :
                        r = self.path_planner.plan_cartesian_goal_and_execute(feedback.marker_name, pt)
                        if not r :
                            rospy.logerr(str("InteractiveControl::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                    else :
                        self.path_planner.clear_goal_target (feedback.marker_name)
                        self.path_planner.create_plan_to_target(feedback.marker_name, pt)
            self.server.applyChanges()
        
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.get_groups() :
                handle = feedback.menu_entry_id
                if handle == self.group_menu_handles[(feedback.marker_name,"Sync To Actual")] :
                    self.reset_group_marker(feedback.marker_name)
                if handle == self.group_menu_handles[(feedback.marker_name,"Execute On Plan")] :
                    state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                    if state == MenuHandler.CHECKED:
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        self.auto_execute[feedback.marker_name] = False
                        self.path_planner.auto_execute[feedback.marker_name] = self.auto_execute[feedback.marker_name]
                    else :
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                        self.auto_execute[feedback.marker_name] = True
                        self.path_planner.auto_execute[feedback.marker_name] = self.auto_execute[feedback.marker_name]
                if handle == self.group_menu_handles[(feedback.marker_name,"Plan On Move")] :
                    state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                    if state == MenuHandler.CHECKED:
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        self.auto_plan[feedback.marker_name] = False
                    else :
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                        self.auto_plan[feedback.marker_name] = True
                if handle == self.group_menu_handles[(feedback.marker_name,"Show Path")] :
                    state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                    if state == MenuHandler.CHECKED:
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        self.path_planner.set_display_mode(feedback.marker_name, "last_point")
                    else :
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                        self.path_planner.set_display_mode(feedback.marker_name, "all_points")
                if handle == self.group_menu_handles[(feedback.marker_name,"Execute")] :
                    r = self.path_planner.execute(feedback.marker_name)
                    if not r :
                        rospy.logerr(str("InteractiveControl::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                if handle == self.group_menu_handles[(feedback.marker_name,"Plan")] :
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = feedback.header
                    pt.pose = feedback.pose
                    if self.auto_execute[feedback.marker_name] :
                        r = self.path_planner.plan_cartesian_goal_and_execute(feedback.marker_name, pt)
                        if not r :
                            rospy.logerr(str("InteractiveControl::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                    else :
                        self.path_planner.clear_goal_target (feedback.marker_name)
                        self.path_planner.create_plan_to_target(feedback.marker_name, pt)
                if handle == self.group_menu_handles[(feedback.marker_name,"Toggle Joint Control")] :
                    self.toggle_posture_control(feedback.marker_name)
        
        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()

if __name__=="__main__":

    rospy.init_node("InteractiveControl")

    robot = rospy.get_param("~robot")
    planner = rospy.get_param("~planner_type")
    group_config_file = rospy.get_param("~group_config_file")

    planner_config_file = rospy.get_param("~planner_config_file", None)   
    tolerance_config_file = rospy.get_param("~tolerance_config_file", None)   
    navigation_frame = rospy.get_param("~navigation_frame", None)
    gripper_service = rospy.get_param("~gripper_service", None)

    control = InteractiveControl(robot, planner, navigation_frame, group_config_file, planner_config_file)

    if gripper_service :
        rospy.loginfo(str("Setting Gripper Service: " + gripper_service))
        control.set_gripper_service(gripper_service)

    if navigation_frame :
        control.navigation_controls.activate_navigation_markers(True)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()
