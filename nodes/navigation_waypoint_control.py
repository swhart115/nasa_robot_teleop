#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math
import threading
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

class NavigationWaypointControl(threading.Thread) :

    def __init__(self, robot, server=None, frame_id="global", tf_listener=None, reference_frame="") :
        super(NavigationWaypointControl,self).__init__()

        self.mutex = threading.Lock()
        self._stop = threading.Event()

        self.robot = robot
        self.frame_id = frame_id
        self.reference_frame = reference_frame

        if server :
            self.server = server
        else :
            self.server = InteractiveMarkerServer(str(self.robot + "_navigation_control"))

        if tf_listener :
            self.tf_listener = tf_listener
        else :
            self.tf_listener = tf.TransformListener()

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_frame = "nav_goal"


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

        if self.reference_frame != "" :
            self.waypoint_menu_options.append("Sync Orientation to Robot")
            self.waypoint_menu_options.append("Sync Orientation to Path")
    
        self.waypoint_menu_options.append("Save Footstep Path")
        # self.waypoint_menu_options.append("Load Footstep Path")

        self.waypoint_menu_options.append("Request Footstep Plan")
        self.waypoint_menu_options.append("Execute Footstep Plan")
        self.waypoint_menu_options.append("Snap PATH To Points")
        self.waypoint_menu_options.append("Swap Start Feet")
        
        self.use_footstep_planner = True

        self.last_waypoint_height = 2.0

        self.running = True  
        self.start()

    def activate_navigation_markers(self, v) :
        self.waypoint_markers_on = v
        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1
        self.add_waypoint(None, p, False)

        if self.reference_frame != "" :
            self.sync_orientation_to_robot()


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
        waypoint.scale = 0.5
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


        cyl = createCylinder(waypoint_id, height=height/2.0, radius=0.2)


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

        self.setup_stored_footstep_menu(waypoint_id)
        self.waypoint_markers.append(waypoint.name)
        self.waypoint_marker_menus[waypoint_id].apply(self.server, waypoint.name)
        self.server.applyChanges()

    def toggle_waypoint_controls(self, feedback) :
        self.server.setPose(feedback.marker_name, feedback.pose)
        self.server.applyChanges()
        
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

    def sync_orientation_to_robot(self) :

        if self.reference_frame :
            self.tf_listener.waitForTransform(self.frame_id, self.reference_frame, rospy.Time(0), rospy.Duration(5.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.frame_id, self.reference_frame, rospy.Time(0))       
            ref_pose = toPose(trans, rot)
        else :
            ref_pose = Pose()
            ref_pose.orientation.w = 1

        for id in self.waypoint_markers :
            ps = Pose()
            n = self.get_waypoint_name(id)
            p = self.server.get(n)  
            ps.position = p.pose.position
            ps.orientation = ref_pose.orientation
            self.server.setPose(n, ps)
        self.server.applyChanges()

    def sync_orientation_to_path(self) :

        if self.reference_frame :
            last_frame = self.reference_frame
        else :
            last_frame = self.frame_id

        self.tf_listener.waitForTransform(last_frame, self.frame_id, rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = self.tf_listener.lookupTransform(last_frame, self.frame_id, rospy.Time(0))       
        ref_pose = toPose(trans, rot)

        for id in self.waypoint_markers :

            n = self.get_waypoint_name(id)
            
            rospy.logwarn(str("NavigationWaypointControl::sync_to_path() -- getting path heading from " + last_frame + " to " + n))
            self.tf_listener.waitForTransform(n, self.frame_id, rospy.Time(0), rospy.Duration(3.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.frame_id, n, rospy.Time(0))       
            waypoint_pose = toPose(trans, rot)

            T1 = fromMsg(ref_pose)
            T2 = fromMsg(waypoint_pose)
            rpy = T2.M.GetRPY()
            
            T2.M = Rotation()

            T = T1.Inverse()*T2
            new_pose = toMsg(T)

            x = new_pose.position.x
            y = new_pose.position.y
            z = new_pose.position.z

            yaw = math.atan2(y,x)
            rpy_new = (rpy[0],rpy[1], yaw)
            q = Rotation.RPY(rpy_new[0],rpy_new[1],rpy_new[2]).GetQuaternion()
            new_pose.orientation.x = q[0]
            new_pose.orientation.y = q[1]
            new_pose.orientation.z = q[2]
            new_pose.orientation.w = q[3]
        
            ps = Pose()
            ps.position = waypoint_pose.position
            ps.orientation = new_pose.orientation
            self.server.setPose(n, ps)

            last_frame = n
            ref_pose = waypoint_pose
            self.server.applyChanges()

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
                rospy.logwarn(str("NavigationWaypointControl::waypoint_menu_callback() -- Request Footstep Plan"))
                print feedback
                self.request_navigation_plan(feedback.marker_name)
            elif handle == self.waypoint_menu_handles["Toggle Full Control"] :
                self.toggle_waypoint_controls(feedback)
            elif handle == self.waypoint_menu_handles["Execute Footstep Plan"] :
                rospy.logwarn(str("NavigationWaypointControl::waypoint_menu_callback() -- Execute Footstep Plan"))
                print feedback
                self.footstep_controls.execute_footstep_path()
            elif handle == self.waypoint_menu_handles["Move Directly"] :
                rospy.logwarn(str("NavigationWaypointControl::waypoint_menu_callback() -- Move Directly"))
                print feedback
                self.direct_move(feedback.marker_name)
            elif handle == self.waypoint_menu_handles["Sync Orientation to Robot"] :
                self.sync_orientation_to_robot()
            elif handle == self.waypoint_menu_handles["Sync Orientation to Path"] :
                self.sync_orientation_to_path()
            elif handle == self.waypoint_menu_handles["Save Footstep Path"] :
                self.save_footstep_path("")
            elif handle == self.waypoint_menu_handles["Snap PATH To Points"] :
                self.snap_path_to_points()
            elif handle == self.waypoint_menu_handles["Swap Start Feet"] :
                self.footstep_controls.swap_footstep(0,1)

    def save_footstep_path(self, filename) :
        self.footstep_controls.set_footstep_filename(filename)
        self.footstep_controls.save_footsteps()
        
    def navigation_marker_callback(self, feedback) :
        self.server.setPose(feedback.marker_name, feedback.pose)
        self.server.applyChanges()
        self.waypoint_poses[feedback.marker_name] = feedback.pose
       
    def snap_path_to_points(self) :
        if self.footstep_controls :
            self.footstep_controls.snap_path_to_points()
        else :
            rospy.logwarn("NavigationControl::snap_path_to_points() -- has no footstep controls")

    def setup_stored_footstep_menu(self, waypoint_id) :
        m = "Load Footstep Path"
        sub_menu_handle = self.waypoint_marker_menus[waypoint_id].insert(m)
        for f in self.footstep_controls.get_footstep_files() :
            self.waypoint_menu_handles[(m,f)] = self.waypoint_marker_menus[waypoint_id].insert(f,parent=sub_menu_handle,callback=self.load_footstep_path_callback)
            self.waypoint_marker_menus[waypoint_id].reApply( self.server )
        self.server.applyChanges()

    def load_footstep_path_callback(self, feedback) :       
        for f in self.footstep_controls.get_footstep_files() :
            if self.waypoint_menu_handles[("Load Footstep Path",f)] == feedback.menu_entry_id :
                rospy.logwarn(str("FootstepControl::load_footsteps_path -- " + f))
                self.footstep_controls.load_footsteps_from_file(f)

    def set_footstep_filename(self, filename) :
        self.footstep_controls.set_footstep_filename(filename)


    def run(self) :
        while self.running :
            try : 

                for goal in self.waypoint_markers :
                # goal = self.waypoint_markers[len(self.waypoint_markers)-1]
                    p = self.server.get(goal)              
                    try :
                        self.tf_broadcaster.sendTransform((p.pose.position.x,p.pose.position.y,p.pose.position.z),
                                              (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w),
                                              rospy.Time.now(), goal, p.header.frame_id)
                    except :
                        rospy.logdebug("NavigationControl::run() -- could not update thread")


                goal = self.waypoint_markers[len(self.waypoint_markers)-1]
                p = self.server.get(goal)              
                try :
                    self.tf_broadcaster.sendTransform((p.pose.position.x,p.pose.position.y,p.pose.position.z),
                                          (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w),
                                          rospy.Time.now(), self.tf_frame, p.header.frame_id)
                except :
                    rospy.logdebug("NavigationControl::run() -- could not update thread")


            except :
                pass
            rospy.sleep(0.1)
      



if __name__=="__main__":


    rospy.init_node("NavigationControl")

    robot = rospy.get_param("~robot", "atlas")
    planner_config_file = rospy.get_param("~planner_config_file", None)   
    navigation_frame = rospy.get_param("~navigation_frame", None)
    robot_reference_frame = rospy.get_param("~robot_reference_frame", "")

    server = InteractiveMarkerServer(str(robot + "_interactive_marker_server"))
    tf_listener = tf.TransformListener()

    from nasa_robot_teleop.planners.atlas_path_planner import AtlasPathPlanner
    path_planner = AtlasPathPlanner(robot, planner_config_file)

    nc = NavigationWaypointControl(robot, server, navigation_frame, tf_listener, robot_reference_frame)
    nc.set_path_planner(path_planner)
    nc.activate_navigation_markers(True)

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        r.sleep()
