#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
import argparse
from copy import deepcopy

# ros messages
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

# helper files
from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.util.marker_helper import *

class NavigationWaypointControl(object) :

    def __init__(self, robot, server, frame="/ground") :

        self.server = server
        self.robot_name = robot
        self.frame = frame

        self.path_planner = None

        self.waypoint_marker_menus = {}
        self.waypoint_menu_handles = {}

        self.waypoint_markers_on = False
        self.waypoint_markers = []
        self.waypoint_stack = []
        self.waypoint_controls = {}
        self.waypoint_poses = {}

        self.footstep_marker_menus = {}
        self.footstep_menu_handles = {}

        self.footstep_markers = {}
        self.footstep_height_controls = {}
        self.footstep_poses = {}

        self.waypoint_menu_options = []
        self.waypoint_menu_options.append("Add Waypoint")
        self.waypoint_menu_options.append("Toggle Full Control")
        self.waypoint_menu_options.append("Delete Waypoint")
        self.waypoint_menu_options.append("Request Plan")
        self.waypoint_menu_options.append("Execute")
        
        self.footstep_menu_options = []
        self.footstep_menu_options.append("Toggle Full Controls")
        
        self.footstep_plan_valid = False

        self.footstep_sub = rospy.Subscriber("/planner/footsteps_in", MarkerArray, self.footstep_callback)
        # self.footstep_pub = rospy.Publisher("/planner/footsteps_out", MarkerArray)
        self.path_pub = rospy.Publisher("/planner/path", Path)      
        

    def activate_navigation_markers(self, v) :
        self.waypoint_markers_on = v
        p = geometry_msgs.msg.Pose()
        p.orientation = Quaternion()
        p.orientation.w = 1
        self.add_waypoint(None, p, False)

    def set_path_planner(self, path_planner) :
        if not isinstance(path_planner, PathPlanner) :
            rospy.logerr("NavigationWaypointControl::set_path_planner() planner not of appropriate type!") 
        else :
            self.path_planner = path_planner

    def add_waypoint(self, parent_name=None, offset=Pose(), replace=False, full_controls=False) :
        
        # add a new waypoint if it's the first
        if len(self.waypoint_markers) == 0:
            self.insert_waypoint(offset, replace, full_controls)
            self.server.applyChanges()
            return True
        else:
            if parent_name:
                node = parent_name
            else:
                node = self.waypoint_markers[0]

            index = self.waypoint_markers.index(node)
            original_size = len(self.waypoint_markers)

            # store any waypoints following the soon to be deleted waypoint
            if index < original_size:
                self.push_waypoints_and_resize(index)

            # insert waypoint into vector and server
            self.insert_waypoint(offset, replace, full_controls)

            # pop any stored waypoints
            if self.waypoint_stack:
                self.pop_waypoints()

            self.server.applyChanges()

            if len(self.waypoint_markers) > original_size:
                return True
            else:
                return False

    def delete_waypoint(self, name):
        index = self.waypoint_markers.index(name)
        original_size = len(self.waypoint_markers)

        # store waypoints
        if index < original_size:
            self.push_waypoints_and_resize(index)

        # delete from vector and server
        if (self.remove_waypoint_from_template(name)):
            # pop any stored waypoints
            if self.waypoint_stack:
                self.pop_waypoints()

            self.server.applyChanges()

        if len(self.waypoint_markers) == original_size - 1:
            return True
        else:
            return False

    def insert_waypoint(self, offset, replace, full_controls=False) :

        key = "NavigationMarker:" + self.get_next_id()

        # init interactivemarker waypoint and adjust offset
        waypoint = InteractiveMarker()
        waypoint.header.frame_id = self.frame
        waypoint.name = key
        waypoint.description = str("NavigationMarker:"+self.get_next_id())
        waypoint.scale = 0.25
        waypoint.pose = offset

        waypoint_id = len(self.waypoint_markers)
        # print "adding waypoint [", key, "] at offset (replace=", replace, ") --- ", offset

        # unless we're replacing a waypoint, slightly adjust the position so we're not inserting
        # directly on top of an existing waypoint
        if not replace:
            waypoint.pose.position.y += 0.1
            # slightly raise z to allow easier selection in rviz
            if waypoint.pose.position.z <= 0.01:
                waypoint.pose.position.z = 0.01

        self.waypoint_poses[key] = waypoint.pose
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
        cyl_control = CreateVisualControlFromMarker(cyl, interaction_mode=InteractiveMarkerControl.MOVE_PLANE)
        waypoint.controls.append(cyl_control)

        self.server.insert(waypoint, self.navigation_marker_callback)
        
        self.waypoint_marker_menus[waypoint_id] = MenuHandler()

        for m in self.waypoint_menu_options :
            self.waypoint_menu_handles[m] = self.waypoint_marker_menus[waypoint_id].insert( m, callback=self.waypoint_menu_callback )

        self.waypoint_markers.append(waypoint.name)
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
        id = len(self.waypoint_markers)
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

        count = len(self.waypoint_markers)
        if name in self.waypoint_markers:
            self.waypoint_markers.remove(name)
        self.server.erase(name)
        # return true if a waypoint was deleted
        if len(self.waypoint_markers) < count:
            return self.server.erase(name)
        else:
            return False

    def pop_waypoints(self):
        # // Now pop each InteractiveMarker from the stack. Grab the pose from the InteractiveMarker
        # // and call addWaypoint(), passing in the offset to create a new marker. The addWaypoint()
        # // method will handle naming of the waypoints correctly, and will also add the waypoint to
        # // the template vector for tracking.
        while self.waypoint_stack:
            waypoint = self.waypoint_stack.pop()
            offset = waypoint.pose
            self.insert_waypoint(offset, replace=True) # need to add full_controls flag here

    def push_waypoints_and_resize(self, index):

        # TODO: The vector of waypoints helps keep track of added waypoints in the template. It looks as
        # if only the "name" field of the waypoint (InteractiveMarker) is getting used. If this is the
        # case, the vector can be a vector of strings containing waypoint names, instead of actual
        # InteractiveMarker objects.
        # Get name of waypoint from vector, and use that to get the InteractiveMarker on the server.
        for i in range(len(self.waypoint_markers)-1, index, -1):
            wp_name = self.waypoint_markers[i]
            interactive_marker = self.server.get(wp_name)
            p = self.waypoint_poses[wp_name]
            # push a copy of the marker onto the stack and delete from the server
            self.waypoint_stack.append(interactive_marker)
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
        name = self.waypoint_markers[index]
        self.waypoint_markers = self.waypoint_markers[:index+1]

    def request_navigation_plan(self, data) :

        waypoint_name = data.marker_name
        rospy.loginfo(str("NavigationWaypointControl::request_navigation_plan() -- requesting plan to " + str(waypoint_name)))

        # clear old feet
        self.clear_footsteps()
        
        waypoints = []

        for id in self.waypoint_markers :
            n = self.get_waypoint_name(id)
            data = self.server.get(n)
                  
            wp = PoseStamped()
            wp.pose = data.pose
            wp.header = data.header
            waypoints.append(wp)
            
            if self.get_waypoint_name(id) == waypoint_name: 
                break        

        if self.path_planner :
            plan = self.path_planner.plan_navigation_path(waypoints)
            # rospy.logwarn("NavigationWaypointControl::request_navigation_plan() -- Turned off request for debuggingg!!")
        else :
            rospy.logwarn("NavigationWaypointControl::request_navigation_plan() no path planner set!")


    def waypoint_menu_callback(self, feedback):
        
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            if handle == self.waypoint_menu_handles["Add Waypoint"] :
                self.add_waypoint(feedback.marker_name, feedback.pose)
            elif handle == self.waypoint_menu_handles["Delete Waypoint"] :
                self.delete_waypoint(feedback.marker_name)
            elif handle == self.waypoint_menu_handles["Request Plan"] :
                self.request_navigation_plan(feedback)
            elif handle == self.waypoint_menu_handles["Toggle Full Control"] :
                self.toggle_waypoint_controls(feedback)
            elif handle == self.waypoint_menu_handles["Execute"] :
                self.execute_footstep_path()


    def navigation_marker_callback(self, feedback) :
        self.waypoint_poses[feedback.marker_name] = feedback.pose
        
    def footstep_callback(self, feedback) :
        self.clear_footsteps()
        self.footstep_array = feedback
        self.translate_feet_to_markers()

    def translate_feet_to_markers(self) :

        marker_names = []

        for m in self.footstep_array.markers :
            if str(m.id) in self.footstep_markers :
                current_marker = self.server.get(str(m.id))
                self.footstep_poses[str(m.id)] = current_marker.pose
                self.server.erase(str(m.id))
            else :
                self.footstep_poses[str(m.id)] = m.pose

            footstep_marker = InteractiveMarker()
            footstep_marker.header.frame_id = deepcopy(m.header.frame_id)
            footstep_marker.header.seq = m.header.seq
            footstep_marker.header.stamp = m.header.stamp

            footstep_marker.name = str(m.id)
            footstep_marker.pose = self.footstep_poses[str(m.id)]
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
                self.translate_feet_to_markers()
            

    def get_foot_poses(self, markers) :
        pose_array = []
        for m in markers:
            p = PoseStamped()
            p.header = m.header
            p.pose = m.pose
            pose_array.append(p)
        return pose_array

    def add_foot_controls(self, feedback) :
        sid = str(feedback.marker_name)
        if sid in self.footstep_height_controls :
            self.footstep_height_controls[sid] = not self.footstep_height_controls[sid]
        else :
            self.footstep_height_controls[sid] = True

    def clear_footsteps(self) :
        print "clearing feet"
        for m in self.footstep_markers.keys():
            #print "erasing marker ", m, " from server"
            self.server.erase(m)
        self.footstep_markers = {}
        self.footstep_height_controls = {}
        self.server.applyChanges()

    def execute_footstep_path(self) :
        print "publishing updated feet"
        # self.footstep_pub.publish(self.footstep_markers)
        if len(self.footstep_markers)>0 and self.footstep_plan_valid :
            step_poses = self.get_foot_poses(self.footstep_markers)
            self.path_planner.execute_navigation_path()
            self.footstep_plan_valid = False


if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Navigation Control')
    parser.add_argument('-r, --robot', dest='robot', help='e.g. r2')
    parser.add_argument('-f, --frame', dest='frame', help='e.g. /world')
    args = parser.parse_args()

    rospy.init_node("NavigationControl")

    server = InteractiveMarkerServer(str(args.robot + "_teleop"))

    nc = NavigationWaypointControl(args.robot, server, args.frame)
    nc.activate_navigation_markers(True)

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        r.sleep()
