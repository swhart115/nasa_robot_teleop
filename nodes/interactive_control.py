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

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *

class InteractiveControl:

    def __init__(self, robot_name, planner_type, navigation_frame, group_config_file, planner_config_file, tolerance_file):

        self.robot_name = robot_name
        self.group_config_file = group_config_file
        self.planner_config_file = planner_config_file
        self.tolerance_file =  tolerance_file
        self.navigation_frame = navigation_frame
        
        self.group_map = []
        self.group_config = {}
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
        if self.navigation_frame and navigation_frame != "":
            rospy.loginfo("InteractiveControl::init() -- setting up NavigationWaypointControl")
            self.navigation_controls = NavigationWaypointControl(self.robot_name, self.server, self.navigation_frame, self.tf_listener)
        else :
            self.navigation_controls = None
        # joint state sub
        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)

        # service helpers
        self.configure_srv = rospy.Service('/interactive_control/configure', InteractiveControlsInterface, self.handle_configure)
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
        elif planner_type == "srv" :
            from nasa_robot_teleop.planners.srv_path_planner import SrvPathPlanner
            self.path_planner = SrvPathPlanner(self.robot_name, self.planner_config_file)
        else :
            rospy.logerr("InteractiveControl() unrecognized planner type!!")
            exit()


        # load the urdf
        self.urdf = self.path_planner.get_urdf_model()

        self.root_frame = self.path_planner.get_robot_planning_frame()
        # set the control frames for all types of groups

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
       
    def parse_config_file(self, config_file) :
        self.config_parser = GroupConfigParser(config_file)
        return self.config_parser.get_group_map()

    def get_tolerances(self, filename) :
        if filename :
            self.tolerances = Tolerance(filename)

    def set_tolerances(self, group, tolerance_mode, tolerance) :
        # print "set tol[", group, "] for ", tolerance_mode, " to ", tolerance
        vals = self.tolerances.get_tolerance_vals(tolerance_mode, tolerance)
        if tolerance_mode == "Position Tolerance" :
            self.path_planner.set_goal_position_tolerances(group, vals)
        elif tolerance_mode == "Angle Tolerance" :
            self.path_planner.set_goal_orientation_tolerances(group, vals)
        else :
            rospy.logerr("InteractiveControl::set_tolerances() -- unknown tolerance mode!")
        # print self.get_tolerance_setting(group, tolerance_mode)


    def get_tolerance_setting(self, group, tolerance_mode) :
        v = [0]*3
        if tolerance_mode == "Position Tolerance" :
            v = self.path_planner.get_goal_position_tolerances(group)
        elif tolerance_mode == "Angle Tolerance" :
            v = self.path_planner.get_goal_orientation_tolerances(group)
            # print "---------------\nGetting tol type for ", group
        m = self.tolerances.get_tolerance_type(tolerance_mode, v)
        return m

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

    def setup_group(self, group, group_type=None) :
        if not group_type :           
            group_type = self.get_group_type(group)

        if not self.path_planner.add_planning_group(group, group_type) :
            self.group_map[group_type].remove(group)
            rospy.logerr(str("InteractiveControl::setup_group() -- planner rejected group: " + group + " of type: " + group_type))
            return False        

        # get stored poses from model
        self.stored_poses[group] = {}
        for state_name in self.path_planner.get_stored_state_list(group) :
            self.stored_poses[group][state_name] = self.path_planner.get_stored_group_state(group, state_name)

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
                    marker.frame_locked = True
                    menu_control.markers.append(marker)
                    idx += 1
            except :
                pass
        # insert marker and menus
        self.markers[group].controls.append(menu_control)
        self.server.insert(self.markers[group], self.process_feedback)


    def initialize_endeffector_group(self, group) : 
        self.setup_endeffector_menus(group)
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        control_frame = self.path_planner.get_control_frame(group)
        ee_links = get_all_child_links(self.urdf, control_frame)
        self.markers[group].header.frame_id = self.path_planner.get_group_base_frame(group)
        self.markers[group].header.stamp = rospy.Time(0)
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
                    end_effector_marker.scale.x *= 1.01
                    end_effector_marker.scale.y *= 1.01
                    end_effector_marker.scale.z *= 1.01
                    end_effector_marker.frame_locked = True
                    menu_control.markers.append(end_effector_marker)
                    idx += 1
            except :
                pass
        self.markers[group].controls.append(menu_control) 

    def remove_group_markers(self, group) :
        self.server.erase(self.markers[group].name)
        del self.markers[group]
        del self.marker_menus[group]
        self.path_planner.remove_group(group)
        self.server.applyChanges()

    def populate_service_response(self) :
        
        resp = InteractiveControlsInterfaceResponse()
        
        for g in self.path_planner.srdf_model.get_groups() :
            resp.group_name.append(g)
        for g in self.markers.keys() :
            try :
                resp.active_group_name.append(g)
                resp.group_type.append(self.get_group_type(g))
                resp.plan_found.append(self.path_planner.plan_generated[g])
            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with basic joint data")
            
            try :
                jm = JointMask()
                jm.mask = self.path_planner.get_joint_mask(g)
                resp.joint_mask.append(jm)
                resp.joint_names.append(self.path_planner.get_joint_map(g))
                # print "Setting joint mask for: ", g, " to: ", resp.joint_mask
            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with joint_mask")
            
            try :
                if g in self.auto_execute.keys() :
                    resp.execute_on_plan.append(self.auto_execute[g])
                else :
                    resp.execute_on_plan.append(False)
            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with auto_execute")
                    
            try :
                if g in self.auto_plan.keys() :
                    resp.plan_on_move.append(self.auto_plan[g])
                else :
                    resp.plan_on_move.append(False)
            except:
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with plan_on_move")

            try :
                if g in self.path_planner.display_modes.keys() :
                    resp.path_visualization_mode.append(self.path_planner.display_modes[g])
                else :
                    resp.path_visualization_mode.append("last_point")
            except:
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with path_visualization_mode")

            try :
                pose_list = StringArray()
                for p in self.stored_poses[g] :
                    pose_list.data.append(p)
                resp.stored_pose_list.append(pose_list)
            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with stored_poses")

            try :               
                ta = ToleranceInfoArray()
                for m in self.tolerances.get_tolerance_modes() :
                    ts = ToleranceInfo()
                    ts.mode = m
                    t = self.get_tolerance_setting(g,m)
                    ts.types.append(t)
                    ta.tolerance_info.append(ts)
                    # print "  ", g, " -- ts: ", m, ": ", t 
                resp.tolerance_setting.append(ta)

            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with tolerance settings")
            

        try :
            for m in self.tolerances.get_tolerance_modes() :

                t = ToleranceInfo()
                t.mode = m
                for tol in self.tolerances.get_tolerances(m) :
                    t.types.append(tol)
                    v = self.tolerances.get_tolerance_vals(m,tol)
                    t.vals.append(Vector3(v[0],v[1],v[2]))
                resp.tolerance.append(t)

        except :
            rospy.logdebug("InteractiveControl::populate_service_response() -- problem with tolerances")

        if self.navigation_controls :

            resp.has_navigation_controls = True

            try :            
                for nwp in self.navigation_controls.get_waypoints() :
                    resp.navigation_waypoint_name.append(nwp)
            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem with nav waypoints settings")

            try :            
                resp.left_foot_first = (self.path_planner.get_start_foot() == "left")
            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem getting start foot")

            try :            
                resp.plan_footsteps = self.navigation_controls.planning_footsteps()
            except :
                rospy.logdebug("InteractiveControl::populate_service_response() -- problem getting start foot")

        else :
            resp.has_navigation_controls = False


        return resp

    def handle_configure(self, req) :
        
        resp = None
        
        if req.action_type == InteractiveControlsInterfaceRequest.GET_INFO :
            resp = self.populate_service_response()
            # print resp
            return resp

        elif req.action_type == InteractiveControlsInterfaceRequest.TOGGLE_POSTURE_CONTROLS :
            for g in req.group_name :
                self.toggle_posture_control(g)

        elif req.action_type == InteractiveControlsInterfaceRequest.EXECUTE_PLAN :
            for g in req.group_name :
                self.reset_group_marker(g)
                if not self.path_planner.execute([g]) :
                    rospy.logerr(str("InteractiveControl::handle_configure() -- failed planner execution for group: " + g))

        elif req.action_type == InteractiveControlsInterfaceRequest.EXECUTE_STORED_POSE :
            for idx in range(len(req.group_name)) :
                g = req.group_name[idx]
                p = req.stored_pose_name[idx]
                self.path_planner.clear_goal_target(g)
                self.path_planner.create_joint_plan([g], [self.stored_poses[g][p]])
                if not self.path_planner.execute([g]): 
                    rospy.logerr(str("InteractiveControl::handle_configure(pose) -- failed stored pose execution for group: " + g))
                self.reset_group_marker(g)
              
        elif req.action_type == InteractiveControlsInterfaceRequest.PLAN_TO_MARKER :
            for idx in range(len(req.group_name)) :
                g = req.group_name[idx]
                if g in self.get_groups('cartesian') :
                    im = self.server.get(g)
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = im.header
                    pt.pose = im.pose

                    try :
                        self.path_planner.display_modes[g] = req.path_visualization_mode[idx]
                    except :
                        pass

                    try :
                        self.auto_execute[g] = req.execute_on_plan[idx]
                    except :
                        pass

                    try :
                        self.auto_plan[g] = req.plan_on_move[idx]
                    except :
                        pass

                    try :
                        self.path_planner.set_joint_mask(g, req.joint_mask[idx].mask)
                    except :
                        pass

                    if self.auto_execute[g] :
                        if not self.path_planner.plan_cartesian_and_execute([g], [[pt]]) :
                            rospy.logerr(str("InteractiveControl::process_feedback(mouse) -- failed planner execution for group: " + g + ". re-synching..."))
                        self.reset_group_marker(g)
                    else :
                        self.path_planner.clear_goal_target(g)
                        self.path_planner.create_path_plan([g], [[pt]])

                    try :
                        for tol in req.tolerance :
                            self.set_tolerances(g,tol.mode,tol.types[0])
                            rospy.loginfo(str("setting ", tol.mode, " tolerance for group ", g, " to ", tol.types[0]))
                    except :
                        pass

                    self.server.applyChanges()


        elif req.action_type == InteractiveControlsInterfaceRequest.SET_JOINT_MAP :
            for idx in range(len(req.group_name)) :
                g = req.group_name[idx]
                try :
                    self.path_planner.set_joint_mask(g, req.joint_mask[idx].mask)
                    joint_names = self.path_planner.get_all_group_joints(g)
                    for jdx in range(len(joint_names)) :
                        jnt = joint_names[jdx]
                        handle = self.group_menu_handles[(g,"Joint Mask", jnt)]
                        if req.joint_mask[idx].mask[jdx]:
                            self.marker_menus[g].setCheckState( handle, MenuHandler.CHECKED )
                        else:
                            self.marker_menus[g].setCheckState( handle, MenuHandler.UNCHECKED )                                                      
                    self.marker_menus[g].reApply( self.server )
                except :
                    pass
            self.server.applyChanges()

        elif req.action_type == InteractiveControlsInterfaceRequest.ADD_GROUP :
            try :
                for idx in range(len(req.group_name)) :
                    gn = req.group_name[idx]
                    gt = req.group_type[idx]
                    if gt in self.group_map.keys() :
                        if not (gn in self.group_map[gt]) :
                            self.group_map[gt].append(gn)
                    else :
                        self.group_map[gt] = [gn]
                    self.setup_group(gn, gt)       
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem adding groups")

        elif req.action_type == InteractiveControlsInterfaceRequest.REMOVE_GROUP :
            for g in req.group_name :
                self.remove_group_markers(g)

        elif req.action_type == InteractiveControlsInterfaceRequest.SET_PLAN_ON_MOVE :
            try :
                for idx in range(len(req.group_name)) :
                    g = req.group_name[idx]
                    self.auto_plan[g] = req.plan_on_move[idx]                                       
                    handle = self.group_menu_handles[(g,"Plan On Move")] 
                    if self.auto_plan[g]:
                        self.marker_menus[g].setCheckState( handle, MenuHandler.CHECKED )
                    else :
                        self.marker_menus[g].setCheckState( handle, MenuHandler.UNCHECKED )
                    self.marker_menus[g].reApply( self.server )
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem setting plan_on_move")
        
        elif req.action_type == InteractiveControlsInterfaceRequest.SET_EXECUTE_ON_PLAN :
            try :
                for idx in range(len(req.group_name)) :
                    g = req.group_name[idx]
                    self.auto_execute[g] = req.execute_on_plan[idx]                                       
                    self.path_planner.auto_execute[g] = self.auto_execute[g]
                    handle = self.group_menu_handles[(g,"Execute On Plan")] 
                    if self.auto_execute[g]:
                        self.marker_menus[g].setCheckState( handle, MenuHandler.CHECKED )
                    else :
                        self.marker_menus[g].setCheckState( handle, MenuHandler.UNCHECKED )
                    self.marker_menus[g].reApply( self.server )
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem setting execute_on_plan")

        elif req.action_type == InteractiveControlsInterfaceRequest.SET_TOLERANCES :
            try :
                for idx in range(len(req.group_name)) :
                    g = req.group_name[idx]                    
                    if g in self.get_groups('cartesian') :
                        try :
                            for tol in req.tolerance :
                                cur_type = self.get_tolerance_setting(g, tol.mode)
                                new_type = tol.types[0]                         
                                if cur_type != new_type :
                                    self.marker_menus[g].setCheckState(self.group_menu_handles[(g,tol.mode,cur_type)], MenuHandler.UNCHECKED )
                                    self.marker_menus[g].setCheckState(self.group_menu_handles[(g,tol.mode,new_type)], MenuHandler.CHECKED )
                                    self.set_tolerances(g,tol.mode,new_type)
                                    self.marker_menus[g].reApply( self.server )
                        except :
                            pass
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem setting tolerances")


        elif req.action_type == InteractiveControlsInterfaceRequest.SET_FIRST_FOOT :
            try :
                if req.left_foot_first :
                    self.path_planner.set_start_foot("left")
                else :
                    self.path_planner.set_start_foot("right")
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem setting start foot")

        elif req.action_type == InteractiveControlsInterfaceRequest.PLAN_FOOTSTEPS_IN_PATH :
            try :
                rospy.logwarn("InteractiveControl::handle_configure() -- Can't plan without footsteps yet ")
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem setting start foot")


        elif req.action_type == InteractiveControlsInterfaceRequest.ADD_NAVIGATION_WAYPOINT :
            try :
                for wp in req.navigation_waypoint_name :
                    self.navigation_controls.add_waypoint(wp)
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem adding nav waypoint")


        elif req.action_type == InteractiveControlsInterfaceRequest.DELETE_NAVIGATION_WAYPOINT :
            try :
                for wp in req.navigation_waypoint_name :
                    self.navigation_controls.delete_waypoint(wp)
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem deleting nav waypoint")

        elif req.action_type == InteractiveControlsInterfaceRequest.PLAN_NAVIGATION_PATH :
            try :
                if len(req.navigation_waypoint_name) > 0 :
                    self.navigation_controls.request_navigation_plan(req.navigation_waypoint_name[0])
                else :
                    rospy.logwarn("InteractiveControl::handle_configure() -- no waypoint goal specified")
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem planning to nav waypoint")


        elif req.action_type == InteractiveControlsInterfaceRequest.EXECUTE_NAVIGATION_PATH :
            try :
                if req.plan_footsteps :
                    self.navigation_controls.footstep_controls.execute_footstep_path()
                else :
                    rospy.logwarn("InteractiveControl::handle_configure() --can only execute footstep plan right now")
            except :
                rospy.logerr("InteractiveControl::handle_configure() -- problem executing nav waypoint")

        self.server.applyChanges()

        resp = self.populate_service_response()
        return resp


    def handle_add_group(self, req) :
        resp = AddGroupResponse()
        g = GroupConfig()
        g.name = req.name
        g.type = req.type
        self.group_map[g.type].append(g.name)

        if len(req.joint_list) == 0 or g.root_frame == "" or g.control_frame == "" :
            pass
            # gc = self.config_parser.get_group_info(g.name, g.type)
            # g.joint_list = gc.joint_list
            # g.root_frame = gc.root_frame
            # g.control_frame = gc.control_frame
        else :
            g.joint_list = req.joint_list
            g.root_frame = req.root_frame
            g.control_frame = req.control_frame
            self.group_map[g.type].append(g.name)

        self.setup_groups()       

        # if self.path_planner.add_planning_group(g.name, g.type) :
        #     resp.id = 0
        #     rospy.loginfo(str("InteractiveControl::handle_add_group() -- added " + req.name + " -- id: " + str(resp.id)))
        #     self.initialize_group_markers(g.name)
        # else :
        #     rospy.logerr(str("InteractiveControl::handle_add_group() -- failed to add " + req.name))

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
            if p == self.tolerances.get_default_tolerance(mode) :
                self.marker_menus[group].setCheckState(self.group_menu_handles[(group,mode,p)], MenuHandler.CHECKED )
                self.set_tolerances(group, mode, p)
            else :
                self.marker_menus[group].setCheckState(self.group_menu_handles[(group,mode,p)], MenuHandler.UNCHECKED )

    def reset_group_marker(self, group, delay=0) :
        rospy.loginfo("InteractiveControl::reset_group_marker()")
        rospy.sleep(delay)
        if self.get_group_type(group) == "cartesian" :
            self.server.setPose(self.markers[group].name, Pose())
        else :
            self.remove_group_markers(group)
            self.initialize_group_markers(group)
        self.server.applyChanges()

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
                            if state == MenuHandler.UNCHECKED:
                                self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                                # print "setting toelrance from menu: ", m, " , ", p, ", ", feedback.marker_name
                                self.set_tolerances(feedback.marker_name, m, p)
                    for p in self.tolerances.get_tolerances(m) :
                        h = self.group_menu_handles[(feedback.marker_name, m, p)]
                        if self.get_tolerance_setting(feedback.marker_name,m) != p :
                            self.marker_menus[feedback.marker_name].setCheckState( h, MenuHandler.UNCHECKED )
                        # else :
                        #     self.marker_menus[feedback.marker_name].setCheckState( h, MenuHandler.CHECKED )

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
                self.path_planner.clear_goal_target(feedback.marker_name)
                self.path_planner.create_joint_plan([feedback.marker_name], [self.stored_poses[feedback.marker_name][p]])
                r = self.path_planner.execute([feedback.marker_name])
                if not r : rospy.logerr(str("InteractiveControl::process_feedback(pose) -- failed execution for group: " + feedback.marker_name + ". re-synching..."))
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
                self.path_planner.create_joint_plan([group], [js])
                r = self.path_planner.execute([group])
                if not r : rospy.logerr(str("InteractiveControl::posture_feedback(pose) -- failed moveitplanner execution for group: " + feedback.marker_name))
            else :
                self.path_planner.clear_goal_target(group)
                self.path_planner.create_joint_plan([group], [js])
        
    def process_feedback(self, feedback) :

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name in self.get_groups('cartesian') :
                if self.auto_plan[feedback.marker_name] :
                    pt = geometry_msgs.msg.PoseStamped()
                    pt.header = feedback.header
                    pt.pose = feedback.pose
                    if self.auto_execute[feedback.marker_name] :
                        r = self.path_planner.plan_cartesian_and_execute([feedback.marker_name], [[pt]])
                        self.reset_group_marker(feedback.marker_name)
                        if not r :
                            rospy.logerr(str("InteractiveControl::process_feedback(mouse) -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                    else :
                        self.path_planner.clear_goal_target(feedback.marker_name)
                        self.path_planner.create_path_plan([feedback.marker_name], [[pt]])
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
                        self.reset_group_marker(feedback.marker_name)
                        r = self.path_planner.execute([feedback.marker_name])
                        if not r :
                            rospy.logerr(str("InteractiveControl::process_feedback() -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                if (feedback.marker_name,"Plan") in self.group_menu_handles:
                    if handle == self.group_menu_handles[(feedback.marker_name,"Plan")] :
                        pt = geometry_msgs.msg.PoseStamped()
                        pt.header = feedback.header
                        pt.pose = feedback.pose
                        if self.auto_execute[feedback.marker_name] :
                            r = self.path_planner.plan_cartesian_and_execute([feedback.marker_name], [[pt]])
                            self.reset_group_marker(feedback.marker_name)
                            if not r :
                                rospy.logerr(str("InteractiveControl::process_feedback() -- failed planner execution for group: " + feedback.marker_name + ". re-synching..."))
                        else :
                            self.path_planner.clear_goal_target(feedback.marker_name)
                            self.path_planner.create_path_plan([feedback.marker_name], [[pt]])
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

    if gripper_service and gripper_service != "" :
       rospy.loginfo(str("Setting Gripper Service: " + gripper_service))
       control.set_gripper_service(gripper_service)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()
