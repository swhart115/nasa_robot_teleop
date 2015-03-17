#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
from copy import deepcopy

# ros messages
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
        self.footstep_height_controls = {}
        self.footstep_poses = {}
       
        self.footstep_menu_options = []
        self.footstep_menu_options.append("Toggle Full Controls")
        # self.footstep_menu_options.append("Execute")
        
        self.footstep_plan_valid = False

        self.footstep_sub = rospy.Subscriber("/planner/footsteps_in", MarkerArray, self.footstep_callback)
        self.path_pub = rospy.Publisher("/planner/path", Path, queue_size=1)      
        
        self.foot_display_offsets = {}
        self.footstep_change_map = []


    def set_path_planner(self, path_planner) :
        if not isinstance(path_planner, PathPlanner) :
            rospy.logerr("FootstepControl::set_path_planner() planner not of appropriate type!") 
        else :
            self.path_planner = path_planner
            self.foot_display_offsets['left'] = self.path_planner.get_foot_display_pose_offset('left')
            self.foot_display_offsets['right'] = self.path_planner.get_foot_display_pose_offset('right')


    def set_footstep_poses(self, poses) :
        self.clear_footsteps()
        self.footstep_change_map = []
        self.translate_poses_to_markers(poses)


    def footstep_callback(self, feedback) :
        self.clear_footsteps()
        self.footstep_array = feedback
        self.translate_feet_to_interactive_markers()


    def translate_poses_to_markers(self, poses) :
        
        self.footstep_array = MarkerArray()
        start_foot = self.path_planner.get_start_foot()
        id = 0 
        if start_foot == "left" :
            indicator = 0
        else :
            indicator = 1

        for p in poses :
            m = Marker()
            m.header = p.header
            m.pose = p.pose

            if id%2 == indicator :
                m.text = "left/" + str(id/2)
            else :
                m.text = "right/" + str(id/2)

            self.footstep_array.markers.append(m)
            id += 1

        self.translate_feet_to_interactive_markers()

    def translate_feet_to_interactive_markers(self) :

        marker_names = []

        for m in self.footstep_array.markers :
            if str(m.id) in self.footstep_markers :
                current_marker = self.server.get(str(m.id))
                self.footstep_poses[str(m.id)] = current_marker.pose
                self.server.erase(str(m.id))
            else :
                self.footstep_poses[str(m.id)] = m.pose

            if "left" in m.text :
                T_off = fromMsg(self.foot_display_offsets['left'])
            else :
                T_off = fromMsg(self.foot_display_offsets['right'])

            T_foot = fromMsg(self.footstep_poses[str(m.id)])
            T_display = T_foot*T_off
            p_display = toMsg(T_display)

            footstep_marker = InteractiveMarker()
            footstep_marker.header.frame_id = deepcopy(m.header.frame_id)
            footstep_marker.header.seq = m.header.seq
            footstep_marker.header.stamp = m.header.stamp

            footstep_marker.name = str(m.id)
            footstep_marker.pose = p_display
            footstep_marker.scale = 0.25
            footstep_marker.description = m.text

            foot = Marker()
            foot.type = Marker.CUBE
            foot.scale.x = 0.2
            foot.scale.y = 0.13
            foot.scale.z = 0.05

            if "left" in m.text :
                foot.color.r = 0.5
                foot.color.g = 0.5
                foot.color.b = 0.75
                foot.color.a = 1.0
            else :
                foot.color.r = 0.5
                foot.color.g = 0.5
                foot.color.b = 0.25
                foot.color.a = 1.0
            foot_control = CreateVisualControlFromMarker(foot, interaction_mode=InteractiveMarkerControl.MOVE_PLANE)

            toe = Marker()
            toe.type = Marker.CUBE
            toe.pose.position.x = foot.scale.x/2.0
            toe.pose.position.y = foot.pose.position.y
            toe.pose.position.z = -(foot.scale.z)/4.0
            toe.scale.x = foot.scale.x/2.0
            toe.scale.y = foot.scale.y/2.0
            toe.scale.z = foot.scale.z/2.0

            if "left" in m.text :
                toe.color.r = 0.5
                toe.color.g = 0.5
                toe.color.b = 0.75
                toe.color.a = 1.0
            else :
                toe.color.r = 0.5
                toe.color.g = 0.5
                toe.color.b = 0.25
                toe.color.a = 1.0
            

            # if m.id%2 == 1:
            #     m.mesh_resource = self.left_foot_mesh
            # else :
            #     m.mesh_resource = self.right_foot_mesh

            # m.type = m.MESH_RESOURCE

            foot_control.markers.append(foot)
            foot_control.markers.append(toe)
            footstep_marker.controls.append(foot_control)
            footstep_marker.controls.append(makeYRotControl())

            if str(m.id) in self.footstep_height_controls.keys() :
                if self.footstep_height_controls[str(m.id)] :
                    print "adding height controls to ", str(m.id)
                    footstep_marker.controls.append(makeYTransControl())
                    footstep_marker.controls.append(makeXRotControl())
                    footstep_marker.controls.append(makeZRotControl())

            self.footstep_markers[footstep_marker.name] = footstep_marker # deepcopy here?

            self.server.insert(footstep_marker, self.footstep_marker_callback)
            
            self.footstep_marker_menus[footstep_marker.name] = MenuHandler()
            for menu_opt in self.footstep_menu_options :
                self.footstep_menu_handles[menu_opt] = self.footstep_marker_menus[footstep_marker.name].insert( menu_opt, callback=self.footstep_marker_callback )

            self.footstep_marker_menus[footstep_marker.name].apply(self.server, footstep_marker.name)
            marker_names.append(footstep_marker.name)


        for n in self.footstep_markers.keys() :
            if n not in marker_names :
                self.server.erase(n)
                del self.footstep_poses[n]
                del self.footstep_markers[n]
                if n in self.footstep_height_controls.keys() :
                    del self.footstep_height_controls[n]
                
        self.footstep_plan_valid = True
        self.server.applyChanges()

    def footstep_marker_callback(self, feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            if handle == self.footstep_menu_handles["Toggle Full Controls"] :
                print "adding height controls for foot: ", feedback.marker_name
                self.add_foot_controls(feedback)
                self.translate_feet_to_interactive_markers()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            print "Moved Foot #: ", feedback.marker_name
            self.footstep_change_map.append(feedback.marker_name)
            

    def get_foot_poses(self, markers, filter=False) :
        pose_array = []
        for id in range(len(markers.keys())):
            if str(id) in self.footstep_change_map or not filter :
                im = markers[str(id)]
                p = PoseStamped()
                p.header = im.header
                p.pose = im.pose
                pose_array.append(p)
        return pose_array

    def add_foot_controls(self, feedback) :
        sid = str(feedback.marker_name)
        if sid in self.footstep_height_controls :
            self.footstep_height_controls[sid] = not self.footstep_height_controls[sid]
        else :
            self.footstep_height_controls[sid] = True

    def clear_footsteps(self) :
        for m in self.footstep_markers.keys():
            self.server.erase(m)
        self.footstep_markers = {}
        self.footstep_height_controls = {}
        self.server.applyChanges()

    def execute_footstep_path(self) :

        rospy.loginfo(str("FootstepControl::execute_footstep_path() -- executing footstep path: " + str(self.footstep_plan_valid)))

        if len(self.footstep_markers)>0 and self.footstep_plan_valid :

            step_poses = self.get_foot_poses(self.footstep_markers, filter=False)

            print "==== INITIAL STEP POSES ==="
            print step_poses
            print "======="
            for s in step_poses :
                T_display = fromMsg(s.pose)
                T_off = fromMsg(self.foot_display_offsets['left']) # THIS IS BAD 
                T_foot = T_display*T_off.Inverse()               
                s.pose = toMsg(T_foot)
            print "==== FINAL STEP POSES ==="
            print step_poses
            print "======="

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
