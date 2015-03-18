#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
import copy
import threading
import tf
import argparse
import PyKDL as kdl

from sys import exit

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
from nasa_robot_teleop.util.marker_helper import *
from nasa_robot_teleop.util.kinematics_util import *

from nasa_robot_teleop.end_effector_helper import *
from nasa_robot_teleop.tolerances import *

# path planner instances
from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.group_config_parser import *
from nasa_robot_teleop.group_config import *

from navigation_waypoint_control import *

class InteractiveControl:

    def __init__(self, robot_name, planner_type, navigation_frame, group_config_file, planner_config_file, tolerance_file):

        self.robot_name = robot_name
        self.group_config_file = group_config_file
        self.planner_config_file = planner_config_file
        self.tolerance_file =  tolerance_file
        self.navigation_frame = navigation_frame
        
        self.group_map = []
        self.tolerances = None
        self.gripper_service = None
        self.config_parser = None

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

        self.end_effector_link_data = {}

        self.stored_poses = {}

        self.auto_plan = {}
        self.auto_execute = {}

        # load in tolerances (if given)
        self.get_tolerances(self.tolerance_file)
      
        # interactive marker server
        self.server = InteractiveMarkerServer(str(self.robot_name + "_interactive_controls_server"))

        # nav control markers
        if self.navigation_frame :
            rospy.loginfo("InteractiveControl::init() -- setting up NavigationWaypointControl")
            self.navigation_controls = NavigationWaypointControl(self.robot_name, self.server, self.navigation_frame, self.tf_listener)

        # joint state sub
        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)

        # service helpers
        self.add_group_srv = rospy.Service('/interactive_control/add_group', AddGroup, self.handle_add_group)
        self.remove_group_srv = rospy.Service('/interactive_control/remove_group', RemoveGroup, self.handle_remove_group)

        # set up menu info
        self.joint_menu_options = []
        self.joint_menu_options.append(("Execute", False))
        self.joint_menu_options.append(("Execute On Plan", True))
        self.joint_menu_options.append(("Show Path", True))       
        self.joint_menu_options.append(("Toggle Joint Control", False))
        self.joint_menu_options.append(("Stored Poses", False))       
        self.joint_menu_options.append(("Joint Mask", False))

        self.cartesian_menu_options = []
        self.cartesian_menu_options.append(("Plan", False))
        self.cartesian_menu_options.append(("Execute", False))
        self.cartesian_menu_options.append(("Plan On Move", True))
        self.cartesian_menu_options.append(("Execute On Plan", True))
        self.cartesian_menu_options.append(("Show Path", True))       
        self.cartesian_menu_options.append(("Toggle Joint Control", False))
        self.cartesian_menu_options.append(("Stored Poses", False))       
        self.cartesian_menu_options.append(("Sync To Actual", False))
        self.cartesian_menu_options.append(("Joint Mask", False))
        if self.tolerances :
            for mode in self.tolerances.get_tolerance_modes() :
                self.cartesian_menu_options.append((mode, False))

        self.endeffector_menu_options = []
        self.endeffector_menu_options.append(("Stored Poses", False))       

        self.menu_options = self.joint_menu_options + self.cartesian_menu_options + self.endeffector_menu_options
        
        # get the groups from the config file
        self.group_map = self.parse_config_file(self.group_config_file)
       
        # planner instantiatation
        if planner_type == "moveit" :
            from nasa_robot_teleop.planners.moveit_path_planner import MoveItPathPlanner
            self.path_planner = MoveItPathPlanner(self.robot_name, self.planner_config_file)
        elif planner_type == "atlas" :
            from nasa_robot_teleop.planners.atlas_path_planner import AtlasPathPlanner
            self.path_planner = AtlasPathPlanner(self.robot_name, self.planner_config_file)
        else :
            rospy.logerr("InteractiveControl() unrecognized planner type!!")
            exit()

        self.root_frame = self.path_planner.get_robot_planning_frame()
        # set the control frames for all types of groups

        # load the urdf
        self.urdf = self.path_planner.get_urdf_model()

        # setup the groups
        self.setup_groups()       


        for n in self.get_groups() :
            self.control_frames[n] = self.path_planner.get_control_frame(n)
        
        # what do we have?  
        self.path_planner.print_basic_info()

        # start up nav stuff
        if self.navigation_frame :
            self.navigation_controls.set_path_planner(self.path_planner)
            self.navigation_controls.activate_navigation_markers(True)

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

    def get_tolerances(self, filename) :
        if filename :
            self.tolerances = Tolerance(filename)

    def set_tolerances(self, group, tolerance_mode, tolerance) :
        vals = self.tolerances.get_tolerance_vals(tolerance_mode, tolerance)
        if tolerance_mode == "Position Tolerance" :
            self.path_planner.set_goal_position_tolerances(group, vals)
        elif tolerance_mode == "Angle Tolerance" :
            self.path_planner.set_goal_orientation_tolerances(group, vals)
        else :
            rospy.logerr("InteractiveControl::set_tolerances() -- unknown tolerance mode!")


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

        # posture stuff
        self.posture_markers[group] = {}
        self.posture_markers_on[group] = False
        
        # add menus to server
        self.marker_menus[group].apply( self.server, group )
        
        self.server.applyChanges()


    def initialize_cartesian_group(self, group) : 
    
        self.setup_cartesian_menus(group)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

        self.markers[group].controls = make6DOFControls()
        self.markers[group].header.frame_id = self.path_planner.get_control_frame(group)
        self.markers[group].scale = 0.2

        # insert marker and menus
        self.markers[group].controls.append(menu_control)
        self.server.insert(self.markers[group], self.process_feedback)
        # self.markers[group].controls.append(menu_control)

        self.reset_group_marker(group)
        
 
    def initialize_joint_group(self, group) :    

        self.setup_joint_menus(group)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

        self.markers[group].header.frame_id = self.path_planner.get_control_frame(group)
        control_frame = self.path_planner.get_control_frame(group)
        jg_links = get_all_child_links(self.urdf, control_frame)
        idx = 0
        for jg_link in jg_links :  # TODO set frames of markers to be actual robot link frames 
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
        # self.markers[group].controls.append(menu_control)
 

    def initialize_endeffector_group(self, group) : 

        self.setup_endeffector_menus(group)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

        control_frame = self.path_planner.get_control_frame(group)
        ee_links = get_all_child_links(self.urdf, control_frame)
        # self.markers[group].header.frame_id = self.path_planner.srdf_model.group_end_effectors[group].parent_link
        self.markers[group].header.frame_id = self.path_planner.srdf_model.get_base_link(group)

        idx = 0
        for ee_link in ee_links :
            try :
                end_effector_marker = get_mesh_marker_for_link(ee_link, self.urdf)
                # TODO set frames of markers to be actual robot link frames
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

        # self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Execute On Plan")], MenuHandler.CHECKED )

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
        rospy.loginfo(str("InteractiveControl::handle_add_group() -- added " + req.name + " -- id: " + str(resp.id)))
        return resp

    def handle_remove_group(self, req) :
        resp = RemoveGroupResponse()
        self.remove_group_markers(req.name)
        resp.success = True
        rospy.loginfo(str("InteractiveControl::handle_remove_group() -- removed " + req.name + " -- " + str(resp.success)))
        return resp

    def set_gripper_service(self, srv) :
        self.gripper_service = srv
        self.path_planner.set_gripper_service(srv)

    def clear_gripper_service(self) :
        self.gripper_service = None
        self.path_planner.clear_gripper_service()

    def toggle_posture_control(self, group) :
        joint_names = self.path_planner.get_all_group_joints(group)
        for idx in range(len(joint_names)) :
            jnt = joint_names[idx]
            lnk = get_joint_child_link(jnt,self.urdf)
            self.joint_group_map[jnt] = group
            if self.posture_markers_on[group] :
                self.server.erase(self.posture_markers[group][lnk].name)
            else :
                joint = self.urdf.joint_map[jnt]
                jnt_marker = makeInteractiveMarker(name=jnt, frame_id=lnk, pose=Pose(), scale=0.2) 
                jnt_marker.name = jnt
                jnt_marker.description = jnt
                jnt_marker.pose.orientation.w = 1
                control = InteractiveMarkerControl()
                control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = axis_to_q(joint.axis)
                control.name = "rotate"
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
                jnt_marker.controls.append(control)
                self.posture_markers[group][lnk] = jnt_marker
                self.server.insert(self.posture_markers[group][lnk], self.posture_feedback)

        self.server.applyChanges()
        self.posture_markers_on[group] = not self.posture_markers_on[group] 

    def setup_joint_menus(self, group) :
        for m,c in self.joint_menu_options :
            if m == "Stored Poses" :
                self.setup_stored_pose_menu(group)
            elif m == "Joint Mask" :
                self.setup_joint_mask_menu(group)
            else :
                self.setup_simple_menu_item(group, m, c)
        self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Show Path")], MenuHandler.CHECKED )

    def setup_endeffector_menus(self, group) :
        for m,c in self.endeffector_menu_options :
            if m == "Stored Poses" :
                self.setup_stored_pose_menu(group)
            else :
                self.setup_simple_menu_item(group, m, c)

    def setup_cartesian_menus(self, group) :

        tolerance_modes = []
        if self.tolerances :
            tolerance_modes = self.tolerances.get_tolerance_modes()

        for m,c in self.cartesian_menu_options :
            if m == "Stored Poses" :
                self.setup_stored_pose_menu(group)
            elif m == "Joint Mask" :
                self.setup_joint_mask_menu(group)
            elif m in tolerance_modes :
                self.setup_tolerance_menu(group, m)
            else :
                self.setup_simple_menu_item(group, m, c)

        self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Show Path")], MenuHandler.CHECKED )


    def setup_simple_menu_item(self, group, item, checkbox=False) :
        self.group_menu_handles[(group,item)] = self.marker_menus[group].insert( item, callback=self.process_feedback )
        if checkbox : self.marker_menus[group].setCheckState( self.group_menu_handles[(group,item)], MenuHandler.UNCHECKED )      

    def setup_stored_pose_menu(self, group) :
        m = "Stored Poses"
        sub_menu_handle = self.marker_menus[group].insert(m)
        for p in self.path_planner.get_stored_state_list(group) :
            self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)

    def setup_joint_mask_menu(self, group) :
        m = "Joint Mask"
        sub_menu_handle = self.marker_menus[group].insert(m) 
        joint_mask = self.path_planner.get_joint_mask(group)
        joint_names = self.path_planner.get_all_group_joints(group)
        for idx in range(len(joint_names)) :
            jnt = joint_names[idx]
            mask_val = joint_mask[idx]
            self.group_menu_handles[(group,m,jnt)] = self.marker_menus[group].insert(jnt,parent=sub_menu_handle,callback=self.joint_mask_callback)   
            if mask_val :
                self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m,jnt)], MenuHandler.CHECKED )        
            else :
                self.marker_menus[group].setCheckState( self.group_menu_handles[(group,m,jnt)], MenuHandler.UNCHECKED )

    def setup_tolerance_menu(self, group, mode) :
        sub_menu_handle = self.marker_menus[group].insert(mode)
        for p in self.tolerances.get_tolerances(mode) :
            self.group_menu_handles[(group,mode,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.tolerance_callback)
            self.marker_menus[group].setCheckState(self.group_menu_handles[(group,mode,p)], MenuHandler.UNCHECKED )

    def reset_group_marker(self, group, delay=0) :
        rospy.sleep(delay)
        self.server.setPose(self.markers[group].name, Pose())

    def get_current_jpos(self, jnt) :
        if jnt in self.joint_data.name :
            idx = self.joint_data.name.index(jnt)
            jpos = self.joint_data.position[idx]
            return jpos
        else :
            return 0

    def joint_state_callback(self, data) :
        self.joint_data = data

    def tolerance_callback(self,feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.get_groups() :
                handle = feedback.menu_entry_id
                for m in self.tolerances.get_tolerance_modes() :
                    for p in self.tolerances.get_tolerances(m) :
                        if handle == self.group_menu_handles[(feedback.marker_name, m, p)] :
                            state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                            if state == MenuHandler.CHECKED:
                                self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            elif state == MenuHandler.UNCHECKED:
                                self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                                self.set_tolerances(feedback.marker_name, m, p)
                        else :
                            h = self.group_menu_handles[(feedback.marker_name, m, p)]
                            self.marker_menus[feedback.marker_name].setCheckState( h, MenuHandler.UNCHECKED )
        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()
                               
    def joint_mask_callback(self, feedback) :
        joint_mask = self.path_planner.get_joint_mask(feedback.marker_name)
        joint_names = self.path_planner.get_all_group_joints(feedback.marker_name)
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.get_groups() :
                handle = feedback.menu_entry_id
                for idx in range(len(joint_names)) :
                    jnt = joint_names[idx]
                    if handle == self.group_menu_handles[(feedback.marker_name,"Joint Mask", jnt)] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            joint_mask[idx] = False
                            self.path_planner.set_joint_mask(feedback.marker_name, joint_mask)
                        else:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )                                                      
                            joint_mask[idx] = True
                            self.path_planner.set_joint_mask(feedback.marker_name, joint_mask)
        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()

    def stored_pose_callback(self, feedback) :
        for p in self.path_planner.get_stored_state_list(feedback.marker_name) :
            if self.group_menu_handles[(feedback.marker_name,"Stored Poses",p)] == feedback.menu_entry_id :
                if True:
                # if self.auto_execute[feedback.marker_name] or self.path_planner.get_group_type(feedback.marker_name) == "endeffector" :
                    self.path_planner.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                    r = self.path_planner.execute(feedback.marker_name)
                    # r = self.path_planner.plan_joint_goal_and_execute(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                    if not r : rospy.logerr(str("InteractiveControl::process_feedback(pose) -- failed moveitplanner execution for group: " + feedback.marker_name + ". re-synching..."))
                else :
                    self.path_planner.clear_goal_target(feedback.marker_name)
                    self.path_planner.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                if self.path_planner.get_group_type(feedback.marker_name) == "cartesian" :
                    self.reset_group_marker(feedback.marker_name)

    def posture_feedback(self, feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.joint_data.name :
                jpos = self.get_current_jpos(feedback.marker_name)
            else :
                rospy.logwarn(str("InteractiveControl::posture_feedback() joint[" + feedback.marker_name + "] not found!"))
                return          
            js = JointState()
            group = self.joint_group_map[feedback.marker_name]
            for j in self.path_planner.get_all_group_joints(group) :
                js.name.append(j)
                if j == feedback.marker_name :
                    q = feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w
                    jnt_axis = self.urdf.joint_map[feedback.marker_name].axis
                    rpy = q_to_rpy(q)
                    if jnt_axis[0] == 1:
                        theta = rpy[0]
                    elif jnt_axis[1] == 1:
                        theta = rpy[1]
                    elif jnt_axis[2] == 1:
                        theta = rpy[2]
                    jgoal = jpos + theta
                    js.position.append(jgoal)   
                else :
                    js.position.append(self.get_current_jpos(j))
            if self.auto_execute[group] :
                self.path_planner.create_joint_plan_to_target(group, js)
                r = self.path_planner.execute(group)
                if not r : rospy.logerr(str("InteractiveControl::posture_feedback(pose) -- failed moveitplanner execution for group: " + feedback.marker_name))
            else :
                self.path_planner.clear_goal_target(group)
                self.path_planner.create_joint_plan_to_target(group, js)
        
    def process_feedback(self, feedback) :

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.get_groups('cartesian') :
                if self.auto_plan[feedback.marker_name] :
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = feedback.header
                    pt.pose = feedback.pose
                    if self.auto_execute[feedback.marker_name] :
                        r = self.path_planner.plan_cartesian_goal_and_execute(feedback.marker_name, pt)
                        self.reset_group_marker(feedback.marker_name)
                        if not r :
                            rospy.logerr(str("InteractiveControl::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                    else :
                        self.path_planner.clear_goal_target(feedback.marker_name)
                        self.path_planner.create_plan_to_target(feedback.marker_name, pt)
            self.server.applyChanges()
        
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.get_groups() :
                handle = feedback.menu_entry_id
                if (feedback.marker_name,"Sync To Actual") in self.group_menu_handles:
                    if handle == self.group_menu_handles[(feedback.marker_name,"Sync To Actual")] :
                        self.reset_group_marker(feedback.marker_name)
                if (feedback.marker_name,"Execute On Plan") in self.group_menu_handles:
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
                if (feedback.marker_name,"Plan On Move") in self.group_menu_handles:
                    if handle == self.group_menu_handles[(feedback.marker_name,"Plan On Move")] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            self.auto_plan[feedback.marker_name] = False
                        else :
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            self.auto_plan[feedback.marker_name] = True
                if (feedback.marker_name,"Show Path") in self.group_menu_handles:
                    if handle == self.group_menu_handles[(feedback.marker_name,"Show Path")] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            self.path_planner.set_display_mode(feedback.marker_name, "last_point")
                        else :
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            self.path_planner.set_display_mode(feedback.marker_name, "all_points")
                if (feedback.marker_name,"Execute") in self.group_menu_handles:
                    if handle == self.group_menu_handles[(feedback.marker_name,"Execute")] :
                        r = self.path_planner.execute(feedback.marker_name)
                        self.reset_group_marker(feedback.marker_name)
                        if not r :
                            rospy.logerr(str("InteractiveControl::process_feedback() -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                if (feedback.marker_name,"Plan") in self.group_menu_handles:
                    if handle == self.group_menu_handles[(feedback.marker_name,"Plan")] :
                        pt = geometry_msgs.msg.PoseStamped()
                        pt.header = feedback.header
                        pt.pose = feedback.pose
                        if self.auto_execute[feedback.marker_name] :
                            r = self.path_planner.plan_cartesian_goal_and_execute(feedback.marker_name, pt)
                            self.reset_group_marker(feedback.marker_name)
                            if not r :
                                rospy.logerr(str("InteractiveControl::process_feedback() -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                        else :
                            self.path_planner.clear_goal_target (feedback.marker_name)
                            self.path_planner.create_plan_to_target(feedback.marker_name, pt)
                if (feedback.marker_name,"Toggle Joint Control") in self.group_menu_handles:
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
    tolerance_file = rospy.get_param("~tolerance_file", None)   
    navigation_frame = rospy.get_param("~navigation_frame", None)
    gripper_service = rospy.get_param("~gripper_service", None)

    control = InteractiveControl(robot, planner, navigation_frame, group_config_file, planner_config_file, tolerance_file)

    if gripper_service :
        rospy.loginfo(str("Setting Gripper Service: " + gripper_service))
        control.set_gripper_service(gripper_service)

    # if navigation_frame :
    #     control.navigation_controls.activate_navigation_markers(True)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()
