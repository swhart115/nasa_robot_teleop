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
from nasa_robot_teleop.util.kinematics_util import *

from footstep_control import *

class NavigationWaypointControl(object) :

    def __init__(self, robot, server=None, frame_id="/global", tf_listener=None) :

        self.robot = robot
        self.frame_id = frame_id

        if server :
            self.server = server
        else :
            self.server = InteractiveMarkerServer(str(self.robot + "_navigation_control"))

        if tf_listener :
            self.tf_listener = tf_listener
        else :
            self.tf_listener = tf.TransformListener()

        self.path_planner = None
        self.footstep_controls = FootstepControl(self.robot, self.server, self.frame_id, self.tf_listener)

        self.waypoint_marker_menus = {}
        self.waypoint_menu_handles = {}

        self.waypoint_markers_on = False
        self.waypoint_markers = []
        self.waypoint_stack = []
        self.waypoint_controls = {}
        self.waypoint_poses = {}

        self.waypoint_menu_options = []
        self.waypoint_menu_options.append("Add Waypoint")
        self.waypoint_menu_options.append("Toggle Full Control")
        self.waypoint_menu_options.append("Delete Waypoint")
        self.waypoint_menu_options.append("Move Directly")
        self.waypoint_menu_options.append("Request Footstep Plan")
        self.waypoint_menu_options.append("Execute Footstep Plan")

        self.use_footstep_planner = True

        self.last_waypoint_height = 2.0

    def activate_navigation_markers(self, v) :
        self.waypoint_markers_on = v
        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1
        self.add_waypoint(None, p, False)


    def set_path_planner(self, path_planner) :
        if not isinstance(path_planner, PathPlanner) :
            rospy.logerr("NavigationWaypointControl::set_path_planner() planner not of appropriate type!") 
        else :
            self.path_planner = path_planner
            self.footstep_controls.set_path_planner(path_planner)


    def add_waypoint(self, parent_name=None, offset=Pose(), replace=False, full_controls=False) :
       
        # add a new waypoint if it's the first
        if len(self.waypoint_markers) == 0:
            self.insert_waypoint(offset, replace, full_controls=full_controls)
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
            self.insert_waypoint(offset, replace, full_controls=full_controls)

            # pop any stored waypoints
            if self.waypoint_stack:
                self.pop_waypoints()
          
            self.server.applyChanges()

            if len(self.waypoint_markers) <= original_size:
                return False

        # do this for adjusting scale of everyone
        self.push_waypoints_and_resize(-1)
        if self.waypoint_stack:
            self.pop_waypoints()
        self.server.applyChanges()

        return True


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

        # do this for adjusting scale of everyone
        self.push_waypoints_and_resize(-1)
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
        waypoint.header.frame_id = self.frame_id
        waypoint.name = key
        waypoint.description = str("NavigationMarker:"+self.get_next_id())
        waypoint.scale = 0.25
        waypoint.pose = offset
        waypoint_id = len(self.waypoint_markers)

        # unless we're replacing a waypoint, slightly adjust the position so we're not inserting
        # directly on top of an existing waypoint
        if not replace:
            waypoint.pose.position.y += 0.1
            # slightly raise z to allow easier selection in rviz
            if waypoint.pose.position.z <= 0.01:
                waypoint.pose.position.z = 0.01
        self.waypoint_poses[key] = waypoint.pose
        
        # set up controls
        translate = CreateNavControl()

        percent = float(len(self.waypoint_markers)+1.0)/float(len(self.waypoint_markers)+len(self.waypoint_stack)+1)

        last_waypoint=(len(self.waypoint_stack)==0)

        if last_waypoint :
            translate.markers.append(createArrow(waypoint_id))
            waypoint.controls.append(translate)
            waypoint.controls.append(makeYRotControl())
            height = self.last_waypoint_height
        else :
            height = self.last_waypoint_height*percent
                
        if full_controls :
            waypoint.controls.append(makeXRotControl())
            waypoint.controls.append(makeZRotControl())
            waypoint.controls.append(makeYTransControl())

        self.waypoint_controls[key] = full_controls

        cyl = createCylinder(waypoint_id, height=height/2.0, radius=0.25)


        def rgb(minimum, maximum, value):
            minimum, maximum = float(minimum), float(maximum)
            ratio = 2 * (value-minimum) / (maximum - minimum)
            b = float(max(0, (1 - ratio)))
            r = float(max(0, (ratio - 1)))
            g = 1.0 - b - r
            return r, g, b

        hm_min = 0.0
        hm_max = 0.5
        hm_range = hm_max-hm_min
        r,g,b = rgb(hm_min,hm_max,hm_min+percent*hm_range)
        cyl.color.r = 0.75#hm_min+r*hm_range
        cyl.color.g = hm_min+g*hm_range
        cyl.color.b = hm_min+b*hm_range
        cyl.color.a = 0.75

        cyl_control = CreateVisualControlFromMarker(cyl, interaction_mode=InteractiveMarkerControl.MOVE_PLANE)
        waypoint.controls.append(cyl_control)

        self.server.insert(waypoint, self.navigation_marker_callback)
        
        # set up menus
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
        if len(self.waypoint_markers) < count:
            return self.server.erase(name)
        else:
            return False


    def pop_waypoints(self):
        # Now pop each InteractiveMarker from the stack. Grab the pose from the InteractiveMarker
        # and call addWaypoint(), passing in the offset to create a new marker. The insert_waypoint()
        # method will handle naming of the waypoints correctly, and will also add the waypoint to
        # the template vector for tracking.
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
        # name = self.waypoint_markers[index]
        self.waypoint_markers = self.waypoint_markers[:index+1]


    def request_navigation_plan(self, waypoint_name) :

        rospy.loginfo(str("NavigationWaypointControl::request_navigation_plan() -- requesting plan to " + str(waypoint_name)))

        # translate IMs to an array of PoseStamped() types
        waypoints = []
        for id in self.waypoint_markers :
            n = self.get_waypoint_name(id)
            p = self.server.get(n)  
            wp = PoseStamped()
            wp.pose = p.pose
            wp.header = p.header
            waypoints.append(wp)
            if self.get_waypoint_name(id) == waypoint_name: 
                break        

        if self.path_planner :
            poses, lift_heights, feet = self.path_planner.plan_navigation_path(waypoints)
            self.footstep_controls.set_footstep_poses(poses, lift_heights, feet)
        else :
            rospy.logwarn("NavigationWaypointControl::request_navigation_plan() no path planner set!")


    def direct_move(self, waypoint_name) :

        rospy.logwarn("NavigationControl::direct_move()")

        ps = PoseStamped()

        for id in self.waypoint_markers :
            n = self.get_waypoint_name(id)
            p = self.server.get(n)  
            ps.pose = p.pose
            ps.header = p.header

        self.path_planner.direct_move(ps)

    def get_waypoints(self) :
        return self.waypoint_markers

    def planning_footsteps(self) :
        return self.use_footstep_planner

    def waypoint_menu_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            if handle == self.waypoint_menu_handles["Add Waypoint"] :
                self.add_waypoint(feedback.marker_name, feedback.pose)
            elif handle == self.waypoint_menu_handles["Delete Waypoint"] :
                self.delete_waypoint(feedback.marker_name)
            elif handle == self.waypoint_menu_handles["Request Footstep Plan"] :
                self.request_navigation_plan(feedback.marker_name)
            elif handle == self.waypoint_menu_handles["Toggle Full Control"] :
                self.toggle_waypoint_controls(feedback)
            elif handle == self.waypoint_menu_handles["Execute Footstep Plan"] :
                self.footstep_controls.execute_footstep_path()
            elif handle == self.waypoint_menu_handles["Move Directly"] :
                self.direct_move(feedback.marker_name)


    def navigation_marker_callback(self, feedback) :
        self.waypoint_poses[feedback.marker_name] = feedback.pose
       


if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Navigation Control')
    parser.add_argument('-r, --robot', dest='robot', help='e.g. r2')
    parser.add_argument('-f, --frame', dest='frame', help='e.g. /world')
    args = parser.parse_args()

    rospy.init_node("NavigationControl")

    server = InteractiveMarkerServer(str(args.robot + "_interactive_marker_server"))

    nc = NavigationWaypointControl(args.robot, server, args.frame)
    nc.activate_navigation_markers(True)

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        r.sleep()
