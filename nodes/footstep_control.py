#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
import random

from copy import deepcopy

# ros messages
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

# helper files
from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.util.marker_helper import *
from nasa_robot_teleop.util.kinematics_util import *


class FootstepControl(object) :

    def __init__(self, robot, server=None, frame="/global") :

        self.robot_name = robot
        self.frame = frame

        if server :
            self.server = server
        else :
            self.server = InteractiveMarkerServer(str(self.robot_name + "_footstep_control"))

        self.path_planner = None

        self.footstep_marker_menus = {}
        self.footstep_menu_handles = {}

        self.footstep_markers = {}
        self.full_footstep_controls = {}
        self.footstep_poses = {}
       
        self.footstep_menu_options = []
        self.footstep_menu_options.append("Toggle Full Controls")
        # self.footstep_menu_options.append("Execute")
        
        self.footstep_plan_valid = False

        self.footstep_sub = rospy.Subscriber("/planner/footsteps_in", MarkerArray, self.footstep_callback)
        self.path_pub = rospy.Publisher("/planner/path", Path, queue_size=1)      
        
        self.foot_display_offsets = {}
        self.footstep_change_map = []
        self.foot_color_map = {}
        self.feet_names = []

        random.seed(rospy.Time.now().secs)


    def set_path_planner(self, path_planner) :
        if not isinstance(path_planner, PathPlanner) :
            rospy.logerr("FootstepControl::set_path_planner() planner not of appropriate type!") 
        else :
            self.path_planner = path_planner
            self.feet_names = self.path_planner.get_feet_names() 
            for foot in self.feet_names :
                self.foot_display_offsets[foot] = self.path_planner.get_foot_display_pose_offset(foot)
                c = ColorRGBA()
                c.r = 0.5
                c.b = 0.5
                c.g = 0.25+0.5*(float(self.feet_names.index(foot))/float(len(self.feet_names)-1))
                c.a = 1.0
                self.foot_color_map[foot] = c
    

    def set_footstep_poses(self, poses) :
        self.clear_footsteps()
        self.translate_poses_to_markers(poses)
        self.create_foot_interactive_markers()


    def footstep_callback(self, feedback) :
        self.clear_footsteps()
        self.footstep_array = feedback
        self.create_foot_interactive_markers()


    def translate_poses_to_markers(self, poses) :       

        self.footstep_array = MarkerArray()
        num_feet = len(self.feet_names)

        start_foot = self.path_planner.get_start_foot()
        start_foot_id = self.feet_names.index(start_foot)

        for id in range(len(poses)) :
            m = Marker()
            m.header = poses[id].header
            m.pose = poses[id].pose

            # this assumes the feet order in the names is the order assoicated 
            # with the input pose array (modulated by the start foot. probably 
            # a bad assumption in general)
            m.text = self.feet_names[(id+start_foot_id)%num_feet] + "/" + str(id/2)            

            self.footstep_array.markers.append(m)


    def create_foot_interactive_markers(self) :

        if len(self.feet_names)==0 :
            rospy.logerr("FootstepControl::create_foot_interactive_markers() -- no foot names found!")
            return

        for m in self.footstep_array.markers :

            # lookup the foot name
            for foot in self.feet_names :
                if foot in m.text : foot_name = foot

            if str(m.id) in self.footstep_markers :
                # get the current pose in case users has moved them
                current_marker = self.server.get(str(m.id))
                self.footstep_poses[str(m.id)] = self.remove_display_offset(current_marker.pose, foot)
                self.server.erase(str(m.id))
            else :
                # get the pose from storage
                self.footstep_poses[str(m.id)] = m.pose

            # get the foot offsets and colors
            foot_color = self.foot_color_map[foot_name]
            foot_pose = self.add_display_offset(self.footstep_poses[str(m.id)], foot_name)

            # create the interactive marker. it will consist of a "foot" and a "toe" 
            footstep_marker = InteractiveMarker()
            footstep_marker.header.frame_id = deepcopy(m.header.frame_id)
            footstep_marker.header.seq = m.header.seq
            footstep_marker.header.stamp = m.header.stamp
            footstep_marker.name = str(m.id)
            footstep_marker.pose = foot_pose
            footstep_marker.scale = 0.25
            footstep_marker.description = m.text

            # create foot marker
            foot = Marker()
            foot.type = Marker.CUBE
            foot.scale.x = 0.2
            foot.scale.y = 0.13
            foot.scale.z = 0.05
            foot.color = foot_color 

            # create toe marker
            toe = Marker()
            toe.type = Marker.CUBE
            toe.pose.position.x = foot.scale.x/2.0
            toe.pose.position.y = foot.pose.position.y
            toe.pose.position.z = -(foot.scale.z)/4.0
            toe.scale.x = foot.scale.x/2.0
            toe.scale.y = foot.scale.y/2.0
            toe.scale.z = foot.scale.z/2.0
            toe.color = foot_color

            # add markers to controls
            foot_control = CreateVisualControlFromMarker(foot, interaction_mode=InteractiveMarkerControl.MOVE_PLANE)
            foot_control.markers.append(foot)
            foot_control.markers.append(toe)
            footstep_marker.controls.append(foot_control)
            footstep_marker.controls.append(makeYRotControl())

            # add "full" controls if requested
            if str(m.id) in self.full_footstep_controls.keys() :
                if self.full_footstep_controls[str(m.id)] :
                    footstep_marker.controls.append(makeYTransControl())
                    footstep_marker.controls.append(makeXRotControl())
                    footstep_marker.controls.append(makeZRotControl())

            # add foot to staorage and server
            self.footstep_markers[footstep_marker.name] = footstep_marker 
            self.server.insert(footstep_marker, self.footstep_marker_callback)

            # add foot menus            
            self.footstep_marker_menus[footstep_marker.name] = MenuHandler()
            for menu_opt in self.footstep_menu_options :
                self.footstep_menu_handles[menu_opt] = self.footstep_marker_menus[footstep_marker.name].insert( menu_opt, callback=self.footstep_marker_callback )
            self.footstep_marker_menus[footstep_marker.name].apply(self.server, footstep_marker.name)
            
        #     marker_names.append(footstep_marker.name)


        # for n in self.footstep_markers.keys() :
        #     if n not in marker_names :
        #         self.server.erase(n)
        #         del self.footstep_poses[n]
        #         del self.footstep_markers[n]
        #         if n in self.full_footstep_controls.keys() :
        #             del self.full_footstep_controls[n]
                
        self.footstep_plan_valid = True
        self.server.applyChanges()

    def footstep_marker_callback(self, feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            if handle == self.footstep_menu_handles["Toggle Full Controls"] :
                self.toggle_foot_controls(feedback)
                self.create_foot_interactive_markers()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            print "Moved Foot #: ", feedback.marker_name
            self.footstep_change_map.append(feedback.marker_name)
            self.update_footstep_markers_from_server()
            

    def get_foot_poses(self, markers, filter=False) :
        pose_array = []      
        for id in range(len(markers.keys())):
            if str(id) in self.footstep_change_map or not filter :
                im = markers[str(id)]

                # lookup the foot name
                for foot in self.feet_names :
                    if foot in im.description : foot_name = foot

                p = PoseStamped()
                p.header = im.header
                p.pose = self.remove_display_offset(im.pose, foot_name)
                pose_array.append(p)
        return pose_array


    def toggle_foot_controls(self, feedback) :
        sid = str(feedback.marker_name)
        if sid in self.full_footstep_controls :
            self.full_footstep_controls[sid] = not self.full_footstep_controls[sid]
        else :
            self.full_footstep_controls[sid] = True


    def clear_footsteps(self) :
        for m in self.footstep_markers.keys():
            self.server.erase(m)
        self.footstep_markers = {}
        self.full_footstep_controls = {}
        self.footstep_change_map = []
        self.server.applyChanges()

    
    def add_display_offset(self, p, foot) :
        T_foot = fromMsg(p)
        T_off = fromMsg(self.foot_display_offsets[foot])
        T_display = T_foot*T_off
        return toMsg(T_display)


    def remove_display_offset(self, p, foot) :
        T_display = fromMsg(p)
        T_off = fromMsg(self.foot_display_offsets[foot])
        T_foot = T_display*T_off.Inverse()               
        return toMsg(T_foot)


    def update_footstep_markers_from_server(self) :       
        for m in self.footstep_markers.keys() :
            self.footstep_markers[m] = self.server.get(m)

    def execute_footstep_path(self) :
        rospy.loginfo(str("FootstepControl::execute_footstep_path() -- executing footstep path: " + str(self.footstep_plan_valid)))
        self.update_footstep_markers_from_server()
        if len(self.footstep_markers)>0 and self.footstep_plan_valid :
            step_poses = self.get_foot_poses(self.footstep_markers, filter=False)
            # print "==== FINAL STEP POSES ==="
            # print step_poses
            # print "======="
            self.path_planner.execute_navigation_plan(step_poses)
            self.footstep_plan_valid = False


if __name__=="__main__":

    rospy.init_node("FootstepControl")

    server = InteractiveMarkerServer(str(args.robot + "_interactive_marker_server"))

    fc = FootstepControl("atlas", server, "/global")
    fc.activate_navigation_markers(True)

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        r.sleep()
