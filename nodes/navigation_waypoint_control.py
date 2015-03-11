#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
import argparse

# ros messages
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker

# helper files
from nasa_robot_teleop.marker_helper import *


class NavigationWaypointControl(object) :

    def __init__(self, robot, server, frame="/ground") :
        self.server = server
        self.robot_name = robot

        self.waypoint_marker_menus = {}
        self.waypoint_menu_handles = {}
        self.navigation_markers_on = False
        self.navigation_markers = []
        self.wp_stack = []
        self.waypoint_controls = {}
        self.navigation_frame = frame

        self.waypoint_menu_options = []
        self.waypoint_menu_options.append("Add Waypoint")
        self.waypoint_menu_options.append("Toggle Full Control")
        self.waypoint_menu_options.append("Delete Waypoint")
        self.waypoint_menu_options.append("Request Plan")

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
            # if index < original_size:
            self.push_waypoints_and_resize(index)

            # insert waypoint into vector and server
            print "Adding wp at: (", offset.position.x, ", ", offset.position.y, ")"
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
        print "deleting waypoint index: ", index
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
        waypoint.header.frame_id = self.navigation_frame
        waypoint.name = key
        waypoint.description = str("NavigationMarker:"+self.get_next_id())
        waypoint.scale = 0.25
        waypoint.pose = offset

        waypoint_id = len(self.navigation_markers)
        # print "adding waypoint [", key, "] at offset (replace=", replace, ") --- ", offset

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

        print "Adding ", waypoint.name, " to storage"
        self.navigation_markers.append(waypoint.name)
        print " now : ", self.navigation_markers
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

        print " -- keys before: ", self.navigation_markers
        count = len(self.navigation_markers)
        if name in self.navigation_markers:
            self.navigation_markers.remove(name)
            print "Removing IM: ", name

        self.server.erase(name)

        print " -- keys after: ", self.navigation_markers

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
            self.insert_waypoint(offset, replace=True) # need to add full_controls flag here

    def push_waypoints_and_resize(self, index):

        # TODO: The vector of waypoints helps keep track of added waypoints in the template. It looks as
        # if only the "name" field of the waypoint (InteractiveMarker) is getting used. If this is the
        # case, the vector can be a vector of strings containing waypoint names, instead of actual
        # InteractiveMarker objects.
        # Get name of waypoint from vector, and use that to get the InteractiveMarker on the server.
        print "======="
        for i in range(len(self.navigation_markers)):
            wp_name = self.navigation_markers[i]
            interactive_marker = self.server.get(wp_name)
            print "wp[", wp_name, "], position = (", interactive_marker.pose.position.x, ", ", interactive_marker.pose.position.y, ")"
        print "=======\n"


        for i in range(len(self.navigation_markers)-1, index, -1):
            wp_name = self.navigation_markers[i]
            interactive_marker = self.server.get(wp_name)
            # push a copy of the marker onto the stack and delete from the server
            self.wp_stack.append(interactive_marker)
            self.server.erase(wp_name)

            print "pushing wp ", wp_name, " to stack with position.x = ", interactive_marker.pose.position.x

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
        
        print "index: ", index
        print "trimming nav markers from "
        print self.navigation_markers
        print "to"
        name = self.navigation_markers[index]
        self.navigation_markers = self.navigation_markers[:index+1]
        # self.navigation_markers.remove(name)
        print self.navigation_markers

        print "======="
        for i in range(len(self.navigation_markers)):
            wp_name = self.navigation_markers[i]
            interactive_marker = self.server.get(wp_name)
            print "wp[", wp_name, "], position = (", interactive_marker.pose.position.x, ", ", interactive_marker.pose.position.y, ")"
        print "=======\n"

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

    def navigation_marker_callback(self, data) :
        # print "nav callback"
        pass


if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Navigation Control')
    parser.add_argument('-r, --robot', dest='robot', help='e.g. r2')
    parser.add_argument('-f, --frame', dest='frame', help='e.g. /world')
    # parser.add_argument('-p, --planner', dest='planner_type', help='e.g. moveit, atlas')
    # parser.add_argument('-m, --manipulator_groups', nargs="*", dest='manipulator_groups', help='space delimited string e.g. "left_arm left_leg right_arm right_leg"')
    # parser.add_argument('-j, --joint_groups', nargs="*", dest='joint_groups', help='space limited string e.g. "head waist"')
    # parser.add_argument('-g, --gripper_service', nargs="*", dest='gripper_service', help='string e.g. "/pr2_gripper_bridge/end_effector_command"')
    # parser.add_argument('-n, --navigation', dest='navigation_markers', help='True | False')
    # parser.add_argument('positional', nargs='*')
    args = parser.parse_args()

    rospy.init_node("NavigationControl")

    server = InteractiveMarkerServer(str(args.robot + "_teleop"))
    nc = NavigationWaypointControl(args.robot, server, args.frame)

    nc.activate_navigation_markers(True)
    
    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        r.sleep()
