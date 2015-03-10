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

# heler files
from nasa_robot_teleop.marker_helper import *
from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.pose_update_thread import *
from nasa_robot_teleop.end_effector_helper import *
from nasa_robot_teleop.tolerances import *

# path planner instances
from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.moveit_path_planner import *
from nasa_robot_teleop.atlas_path_planner import *

class RobotTeleop:

    def __init__(self, robot_name, planner_type, config_package, manipulator_group_names, joint_group_names):
        self.robot_name = robot_name
        self.group_names = []
        self.manipulator_group_names = manipulator_group_names
        self.joint_group_names = joint_group_names
        self.tf_listener = tf.TransformListener()
        self.joint_data = sensor_msgs.msg.JointState()
        
        self.markers = {}
        self.marker_menus = {}
        self.waypoint_marker_menus = {}
        self.group_pose_data = {}
        self.control_frames = {}
        self.stored_poses = {}
        self.group_menu_handles = {}
        self.waypoint_menu_handles = {}
        self.pose_update_thread = {}
        self.pose_store = {}
        self.auto_plan = {}
        self.auto_execute = {}
        self.end_effector_link_data = {}
        self.position_tolerance_modes = {}
        self.orientation_tolerance_modes = {}
        self.group_position_tolerance_mode = {}
        self.group_orientation_tolerance_mode = {}
        self.gripper_service = None
        self.navigation_markers_on = False
        self.navigation_markers = []
        self.wp_stack = []
        self.waypoint_controls = {}
        # interactive marker server
        self.server = InteractiveMarkerServer(str(self.robot_name + "_teleop"))
        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)

        # set up MoveIt! interface
        if config_package=="" :
            config_package =  str(self.robot_name + "_moveit_config")

        if planner_type == "moveit" :
            self.path_planner = MoveItPathPlanner(self.robot_name,config_package)
        elif planner_type == "atlas" :
            self.path_planner = AtlasPathPlanner(self.robot_name, config_package)
        else :
            rospy.logerr("RobotTeleop() unrecognized planner type!!")
            exit()

        self.root_frame = self.path_planner.get_robot_planning_frame()
        
        self.use_tolerances = self.path_planner.uses_tolerances()

        # add user specified groups
        for n in self.manipulator_group_names :
            if self.path_planner.add_planning_group(n, group_type="manipulator") :
                self.group_names.append(n)
            else :
                self.manipulator_group_names.remove(n)
                rospy.logwarn(str("RobotTeleop::init() -- not adding manipulator group: " + n))
        
        for n in self.joint_group_names :
            print "teleop adding joint group: ", n
            if self.path_planner.add_planning_group(n, group_type="joint") :
                self.group_names.append(n)
            else :
                self.joint_group_names.remove(n)
                rospy.logwarn(str("RobotTeleop::init() -- not adding joint group: " + n))
        
        # append group list with auto-found end effectors
        for n in self.path_planner.get_end_effector_names() :
            self.group_names.append(n)

        # set the control frames for all types of groups
        for n in self.group_names :
            self.control_frames[n] = self.path_planner.get_control_frame(n)

        # what do we have?
        self.path_planner.print_basic_info()

        # set up menu info
        self.menu_options = []
        self.menu_options.append(("Stored Poses", False))
        self.menu_options.append(("Sync To Actual", False))
        self.menu_options.append(("Plan", False))
        self.menu_options.append(("Plan On Move", True))
        self.menu_options.append(("Execute", False))
        self.menu_options.append(("Execute On Plan", True))
        self.menu_options.append(("Show Path", True))       
        # self.menu_options.append(("Turn on Joint Control", True))

        self.waypoint_menu_options = []
        self.waypoint_menu_options.append("Add Waypoint")
        self.waypoint_menu_options.append("Toggle Full Control")
        self.waypoint_menu_options.append("Delete Waypoint")
        self.waypoint_menu_options.append("Request Plan")

        # get stored poses from model
        for group in self.group_names :
            self.stored_poses[group] = {}
            for state_name in self.path_planner.get_stored_state_list(group) :
                self.stored_poses[group][state_name] = self.path_planner.get_stored_group_state(group, state_name)

        # get tolerance modes and append it to each group (probably will all be the same)
        for group in self.group_names :
            if self.use_tolerances :
                self.position_tolerance_modes[group] = self.path_planner.tolerances.get_tolerance_modes('PositionTolerance')
                self.orientation_tolerance_modes[group] = self.path_planner.tolerances.get_tolerance_modes('OrientationTolerance')
                self.path_planner.set_goal_position_tolerance_mode(group, None)
                self.path_planner.set_goal_orientation_tolerance_mode(group, None)
        
        # start update threads for manipulators
        for n in self.manipulator_group_names :
            self.start_pose_update_thread(n)

        # Create EndEffectorHelper objects to help with EE displays
        for n in self.path_planner.get_end_effector_names() :
            self.end_effector_link_data[n] = EndEffectorHelper(self.robot_name, n, self.path_planner.get_control_frame(n), self.tf_listener)
            self.end_effector_link_data[n].populate_data(self.path_planner.get_group_links(n), self.path_planner.get_urdf_model(), self.path_planner.get_srdf_model())

        # set group to display only last point in path by default (can turn on full train from menu)
        for group in self.group_names :
            self.path_planner.set_display_mode(group, "all_points")

        # initialize markers
        self.initialize_group_markers()


    def initialize_group_markers(self) :

        self.group_menu_handles = {}
        self.marker_menus = {}

        urdf = self.path_planner.get_urdf_model()

        for group in self.path_planner.get_group_names() :

            self.auto_execute[group] = False
            self.auto_plan[group] = False
            self.markers[group] = InteractiveMarker()
            self.markers[group].name = group
            self.markers[group].description = group
            self.marker_menus[group] = MenuHandler()

            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

            if self.path_planner.get_group_type(group) == "manipulator" :

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
                self.reset_group_marker(group)

            elif  self.path_planner.get_group_type(group) == "joint" :

                self.markers[group].header.frame_id = self.path_planner.get_control_frame(group)
                control_frame = self.path_planner.get_control_frame(group)
                jg_links = urdf.get_all_child_links(control_frame)
            
                idx = 0
                # print "FOUND JG-LINKS FOR GROUP[", group,"]: ", jg_links
                for jg_link in jg_links :
                    try :
                        marker = get_mesh_marker_for_link(jg_link, urdf)

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
                            # print "ADDING MESH FOR LINK: ", jg_link
                            # print marker
                    except :
                        pass


                # original 
                # mesh = self.path_planner.get_control_mesh(group)
                # pose = self.path_planner.get_control_mesh_pose_offset(group)
                # scale = self.path_planner.get_control_mesh_scale(group)
                # marker = makeMesh( self.markers[group], mesh, pose, [s*1.02 for s in scale], alpha=0.1 )
                # menu_control.markers.append( marker )

                # insert marker and menus
                self.markers[group].controls.append(menu_control)
                self.server.insert(self.markers[group], self.process_feedback)

            elif self.path_planner.get_group_type(group) == "endeffector" :
                control_frame = self.path_planner.get_control_frame(group)
                ee_links = urdf.get_all_child_links(control_frame)
                self.markers[group].header.frame_id = self.path_planner.srdf_model.group_end_effectors[group].parent_link
                
                # original 
                # end_effector_marker = self.end_effector_link_data[group].get_current_position_marker(self.markers[group].header.frame_id, scale=1.02, color=(1,1,1,0.1))
                # menu_control.markers.append( end_effector_marker )

                idx = 0
                for ee_link in ee_links :
                    try :
                        end_effector_marker = get_mesh_marker_for_link(ee_link, urdf)

                        if end_effector_marker != None :
                            end_effector_marker.color.r = 1
                            end_effector_marker.color.g = 1
                            end_effector_marker.color.b = 1
                            end_effector_marker.color.a = 0.1
                            end_effector_marker.id = idx
                            menu_control.markers.append(end_effector_marker)
                            idx += 1
                            # print "ADDING MESH FOR LINK: ", ee_link
                    except :
                        pass

                # insert marker and menus
                self.markers[group].controls.append(menu_control)
                self.server.insert(self.markers[group], self.process_feedback)
                self.auto_execute[group] = True

            # Set up stored pose sub menu
            self.setup_sub_menus(group)

            if self.path_planner.get_group_type(group) == "endeffector" :
                self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Execute On Plan")], MenuHandler.CHECKED )

            self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Show Path")], MenuHandler.CHECKED )
            self.marker_menus[group].setCheckState( self.group_menu_handles[(group,"Plan On Move")], MenuHandler.UNCHECKED )
            # add menus to server
            self.marker_menus[group].apply( self.server, group )
            self.server.applyChanges()
       
    def set_gripper_service(self, srv) :
        self.gripper_service = srv
        self.path_planner.set_gripper_service(srv)

    def clear_gripper_service(self) :
        self.gripper_service = None
        self.path_planner.clear_gripper_service()

    def activate_navigation_markers(self, v) :
        self.navigation_markers_on = v

        p = geometry_msgs.msg.Pose()
        p.orientation = Quaternion()
        p.orientation.w = 1

        self.add_waypoint(None, p, False)


    def add_waypoint(self, parent_name=None, offset=Pose(), replace=False, full_controls=False) :

        # add a new waypoint if it's the first
        if len(self.navigation_markers) == 0:
            self.insert_waypoint(offset, replace, full_controls)
            self.server.applyChanges()
            return True
        else:
            if parent_name:
                node = parent_name
            else:
                node = self.navigation_markers[0]

            index = self.navigation_markers.index(node)
            original_size = len(self.navigation_markers)

            # store any waypoints following the soon to be deleted waypoint
            if index < original_size:
                self.push_waypoints_and_resize(index)

            # insert waypoint into vector and server
            self.insert_waypoint(offset, replace, full_controls)

            # pop any stored waypoints
            if self.wp_stack:
                self.pop_waypoints()

            # if len(self.navigation_markers) > 1:
            #     self.createPaths()

            self.server.applyChanges()

            if len(self.navigation_markers) > original_size:
                return True
            else:
                return False

    def delete_waypoint(self, name):
        index = self.navigation_markers.index(name)
        original_size = len(self.navigation_markers)

        # store waypoints
        if index < original_size:
            self.push_waypoints_and_resize(index)

        # delete from vector and server
        if (self.remove_waypoint_from_template(name)):
            # pop any stored waypoints
            if self.wp_stack:
                self.pop_waypoints()

            # if len(self.navigation_markers) > 1:
            #     self.createPaths()

            self.server.applyChanges()

        if len(self.navigation_markers) == original_size - 1:
            return True
        else:
            return False

    def insert_waypoint(self, offset, replace, full_controls=False) :

        key = "NavigationMarker:" + self.get_next_id()

        # init interactivemarker waypoint and adjust offset
        waypoint = InteractiveMarker()
        waypoint.header.frame_id = "/ground"
        waypoint.name = key
        waypoint.description = str("NavigationMarker:"+self.get_next_id())
        waypoint.scale = 0.25
        waypoint.pose = offset


        # print "adding waypoint [", key, "] at offset (replace=", replace, ") --- ", offset

        waypoint_id = len(self.navigation_markers)
        # unless we're replacing a waypoint, slightly adjust the position so we're not inserting
        # directly on top of an existing waypoint
        if not replace:
            waypoint.pose.position.y += 0.1
            # slightly raise z to allow easier selection in rviz
            if waypoint.pose.position.z <= 0.01:
                waypoint.pose.position.z = 0.01

        # create a control to translate in XY plane and add arrow visualization.
        # also add rotate Z control
        translate = CreateNavControl()
        translate.markers.append(createArrow(waypoint_id))
        waypoint.controls.append(translate)

        waypoint.controls.append(makeYRotControl())

        if full_controls :
            waypoint.controls.append(makeXRotControl())
            waypoint.controls.append(makeZRotControl())
            waypoint.controls.append(makeYTransControl())
        
        self.waypoint_controls[key] = full_controls

        cyl = createCylinder(waypoint_id, height=1.0, radius=0.25)
        cyl_control = CreateVisualControlFromMarker(cyl)#,interaction_mode=InteractiveMarkerControl.MOVE_PLANE)
        waypoint.controls.append(cyl_control)
        # height_control = makeYTransControl()
        # waypoint.controls.append(height_control)

        # radius = createCylinder(waypoint_id)
        # cyl_control = CreateVisualControlFromMarker(radius)
        # waypoint.controls.append(cyl_control)

        self.server.insert(waypoint, self.navigation_marker_callback)
        
        self.waypoint_marker_menus[waypoint_id] = MenuHandler()

        for m in self.waypoint_menu_options :
            self.waypoint_menu_handles[m] = self.waypoint_marker_menus[waypoint_id].insert( m, callback=self.waypoint_menu_callback )

        self.navigation_markers.append(waypoint.name)

        self.waypoint_marker_menus[waypoint_id].apply(self.server, waypoint.name)

        self.server.applyChanges()

    def toggle_waypoint_controls(self, feedback) :

        name = feedback.marker_name
        
        if not self.waypoint_controls[name] :
            self.delete_waypoint(name)
            self.add_waypoint(offset=feedback.pose, replace=True, full_controls=True)
            self.waypoint_controls[name] = True
        else :
            self.delete_waypoint(name)
            self.add_waypoint(offset=feedback.pose, replace=True, full_controls=False)
            self.waypoint_controls[name] = False
        
        self.server.applyChanges()

    def get_next_id(self):
        id = len(self.navigation_markers)
        if id < 10:
            str_id = "0" + str(id)
        else:
            str_id = str(id)
        return str_id

    def get_waypoint_name(self, id):
        if id < 10:
            str_id = "0" + str(id)
        else:
            str_id = str(id)
        return str_id

    def remove_waypoint_from_template(self, name):
        count = len(self.navigation_markers)
        if name in self.navigation_markers:
            self.navigation_markers.remove(name)

        self.server.erase(name)

        # return true if a waypoint was deleted
        if len(self.navigation_markers) < count:
            return self.server.erase(name)
        else:
            return False

    def pop_waypoints(self):
        # // Now pop each InteractiveMarker from the stack. Grab the pose from the InteractiveMarker
        # // and call addWaypoint(), passing in the offset to create a new marker. The addWaypoint()
        # // method will handle naming of the waypoints correctly, and will also add the waypoint to
        # // the template vector for tracking.
        while self.wp_stack:
            waypoint = self.wp_stack.pop()
            offset = waypoint.pose
            self.insert_waypoint(offset, True)

    def push_waypoints_and_resize(self, index):

        # TODO: The vector of waypoints helps keep track of added waypoints in the template. It looks as
        # if only the "name" field of the waypoint (InteractiveMarker) is getting used. If this is the
        # case, the vector can be a vector of strings containing waypoint names, instead of actual
        # InteractiveMarker objects.
        # Get name of waypoint from vector, and use that to get the InteractiveMarker on the server.
        for i in range(len(self.navigation_markers)-1, index, -1):
            wp_name = self.navigation_markers[i]
            interactive_marker = self.server.get(wp_name)
            # push a copy of the marker onto the stack and delete from the server
            self.wp_stack.append(interactive_marker)
            self.server.erase(wp_name)

        # // Now resize vector to the index.
        # // Because our vector of waypoints are both numbered sequentially and don't skip numbers,
        # // We can just resize the vector to the index.
        # // Example:
        # // for vector {0, 1, 2, 3, 4, 5 ,6 ,7 ,8, 9}
        # // we delete index 5, and pop remaining into a stack
        # //   stack {9, 8, 7, 6} <- top
        # // we want the new vector to be
        # //   vector {0, 1, 2, 3, 4}
        # // So we can resize it using the index. This seems dirty. Is this dirty?
        self.navigation_markers = self.navigation_markers[:index+1]


    def request_navigation_plan(self, data) :

        waypoint_name = data.marker_name
        rospy.loginfo(str("RobotTeleop::request_navigation_plan() -- requesting plan to " + str(waypoint_name)))
        
        waypoints = []

        for id in self.navigation_markers :

            print " ID: ", id
            n = self.get_waypoint_name(id)
            print " NAME: ", n
            data = self.server.get(n)
            print " POSE: ",
            print data.pose
            # print data
                  
            wp = PoseStamped()
            wp.pose = data.pose
            wp.header = data.header
            waypoints.append(wp)
            
            if self.get_waypoint_name(id) == waypoint_name: 
                break        

        plan = self.path_planner.plan_navigation_path(waypoints)

    def setup_sub_menus(self, group) :
        for m,c in self.menu_options :
            if m == "Stored Poses" :
                sub_menu_handle = self.marker_menus[group].insert(m)
                for p in self.path_planner.get_stored_state_list(group) :
                    self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)
            elif m == "Position Tolerance" :
                sub_menu_handle = self.marker_menus[group].insert(m)
                for p in self.position_tolerance_modes[group] :
                    self.group_menu_handles[(group,m,p)] = self.marker_menus[group].insert(p,parent=sub_menu_handle,callback=self.position_tolerance_callback)
                    self.marker_menus[group].setCheckState(self.group_menu_handles[(group,m,p)], MenuHandler.UNCHECKED )
            elif m == "Angle Tolerance" :
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
                # What?! do it again? Why? Huh?!
                self.server.setPose(self.markers[group].name, self.group_pose_data[group])
                self.server.applyChanges()
                _marker_valid = True
            rospy.sleep(0.1)

    def joint_state_callback(self, data) :
        self.joint_data = data

    def position_tolerance_callback(self,feedback) :
        print ""
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.group_names :
                self.group_position_tolerance_mode[feedback.marker_name] = "FULL"
                self.path_planner.set_goal_position_tolerance_mode(feedback.marker_name, "FULL")
                handle = feedback.menu_entry_id
                for p in self.position_tolerance_modes[feedback.marker_name] :
                    if handle == self.group_menu_handles[(feedback.marker_name,"Position Tolerance", p)] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            print " setting (", feedback.marker_name,", Position Tolerance, ", p, ") to unchecked"
                        elif state == MenuHandler.UNCHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            print " setting (", feedback.marker_name, ", Position Tolerance, ", p, ") to checked"
                            self.group_position_tolerance_mode[feedback.marker_name] = p
                            self.path_planner.set_goal_position_tolerance_mode(feedback.marker_name, p)
                    else :
                        h = self.group_menu_handles[(feedback.marker_name,"Position Tolerance", p)]
                        self.marker_menus[feedback.marker_name].setCheckState( h, MenuHandler.UNCHECKED )
                        # print "    unchecking (", feedback.marker_name, ", Position Tolerance, ", p, ")"
        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()

    def orientation_tolerance_callback(self,feedback) :
        print ""
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.group_names :
                handle = feedback.menu_entry_id
                self.group_orientation_tolerance_mode[feedback.marker_name] = "FULL"
                self.path_planner.set_goal_orientation_tolerance_mode(feedback.marker_name, "FULL")
                for p in self.orientation_tolerance_modes[feedback.marker_name] :
                    if handle == self.group_menu_handles[(feedback.marker_name,"Angle Tolerance", p)] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            print " setting (", feedback.marker_name, ", Angle Tolerance, ", p, ") to unchecked"
                        elif state == MenuHandler.UNCHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            print " setting (", feedback.marker_name, ", Angle Tolerance, ", p, ") to checked"
                            self.group_orientation_tolerance_mode[feedback.marker_name] = p
                            self.path_planner.set_goal_orientation_tolerance_mode(feedback.marker_name, p)
                    else :
                        h = self.group_menu_handles[(feedback.marker_name,"Angle Tolerance", p)]
                        self.marker_menus[feedback.marker_name].setCheckState( h, MenuHandler.UNCHECKED )
                        # print "    unchecking (", feedback.marker_name, ", Angle Tolerance, ", p, ")"
                
        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()
                                 
    def joint_mask_callback(self, feedback) :
        if self.path_planner.has_joint_mask(feedback.marker_name) and self.path_planner.has_joint_map(feedback.marker_name) :
            joint_map = self.path_planner.get_joint_map(feedback.marker_name)
            joint_mask = self.path_planner.get_joint_mask(feedback.marker_name)
            if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
                if feedback.marker_name in self.group_names :
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
                    if not r : rospy.logerr(str("RobotTeleop::process_feedback(pose) -- failed moveitplanner execution for group: " + feedback.marker_name + ". re-synching..."))
                else :
                    self.path_planner.clear_goal_target(feedback.marker_name)
                    self.path_planner.create_joint_plan_to_target(feedback.marker_name, self.stored_poses[feedback.marker_name][p])
                if self.path_planner.get_group_type(feedback.marker_name) == "manipulator" :
                    rospy.sleep(3)
                    self.reset_group_marker(feedback.marker_name)

    def navigation_marker_callback(self, data) :
        # print "nav callback"
        pass

    def waypoint_menu_callback(self, feedback):
        
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            print handle
            if handle == self.waypoint_menu_handles["Add Waypoint"] :
                self.add_waypoint(feedback.marker_name, feedback.pose)
            elif handle == self.waypoint_menu_handles["Delete Waypoint"] :
                self.delete_waypoint(feedback.marker_name)
            elif handle == self.waypoint_menu_handles["Request Plan"] :
                self.request_navigation_plan(feedback)
            elif handle == self.waypoint_menu_handles["Toggle Full Control"] :
                self.toggle_waypoint_controls(feedback)

    def process_feedback(self, feedback) :

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            if feedback.marker_name in self.manipulator_group_names :
                self.pose_store[feedback.marker_name] = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.manipulator_group_names :
                if self.auto_plan[feedback.marker_name] :
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = feedback.header
                    pt.pose = feedback.pose
                    print "=====================\nFEEDBACK\n"
                    print feedback
                    print "=====================\n"
                    if self.auto_execute[feedback.marker_name] :
                        r = self.path_planner.plan_cartesian_goal_and_execute(feedback.marker_name, pt)
                        if not r :
                            rospy.logerr(str("RobotTeleop::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                    else :
                        self.path_planner.clear_goal_target (feedback.marker_name)
                        self.path_planner.create_plan_to_target(feedback.marker_name, pt)

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.marker_name in self.group_names :
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
                        rospy.logerr(str("RobotTeleop::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                if handle == self.group_menu_handles[(feedback.marker_name,"Plan")] :
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = feedback.header
                    pt.pose = feedback.pose
                    if self.auto_execute[feedback.marker_name] :
                        r = self.path_planner.plan_cartesian_goal_and_execute(feedback.marker_name, pt)
                        if not r :
                            rospy.logerr(str("RobotTeleop::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                    else :
                        self.path_planner.clear_goal_target (feedback.marker_name)
                        self.path_planner.create_plan_to_target(feedback.marker_name, pt)

        self.marker_menus[feedback.marker_name].reApply( self.server )
        self.server.applyChanges()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Robot Teleop')
    parser.add_argument('-r, --robot', dest='robot', help='e.g. r2')
    parser.add_argument('-c, --config', dest='config', help='e.g. r2_fullbody_moveit_config')
    parser.add_argument('-p, --planner', dest='planner_type', help='e.g. moveit, atlas')
    parser.add_argument('-m, --manipulator_groups', nargs="*", dest='manipulator_groups', help='space delimited string e.g. "left_arm left_leg right_arm right_leg"')
    parser.add_argument('-j, --joint_groups', nargs="*", dest='joint_groups', help='space limited string e.g. "head waist"')
    parser.add_argument('-g, --gripper_service', nargs="*", dest='gripper_service', help='string e.g. "/pr2_gripper_bridge/end_effector_command"')
    parser.add_argument('-n, --navigation', dest='navigation_markers', help='True | False')
    parser.add_argument('positional', nargs='*')
    args = parser.parse_args()

    rospy.init_node("RobotTeleop")

    robot = RobotTeleop(args.robot, args.planner_type, args.config, args.manipulator_groups, args.joint_groups)

    if args.gripper_service :
        rospy.loginfo(str("Setting Gripper Service: " + args.gripper_service[0]))
        robot.set_gripper_service(args.gripper_service[0])

    if args.navigation_markers :
        robot.activate_navigation_markers(True)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        r.sleep()
