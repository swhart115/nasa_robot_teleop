#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

roslib.load_manifest("auto_walker")

import math
import random
import tf
import pickle
import PyKDL as kdl

from copy import deepcopy

# ros messages
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from auto_walker.msg import *
import auto_walker

# helper files
from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.util.marker_helper import *
from nasa_robot_teleop.util.kinematics_util import *

class FootstepStorage(object) :
    def __init__(self) :
        self.feet = None
        self.lift_heights = None
        self.step_poses = None


class FootstepControl(object) :

    def __init__(self, robot, server=None, frame_id="global", tf_listener=None) :

        self.robot_name = robot
        self.frame_id = frame_id

        if server :
            self.server = server
        else :
            self.server = InteractiveMarkerServer(str(self.robot_name + "_footstep_control"))

        if tf_listener :
            self.tf_listener = tf_listener
        else :
            self.tf_listener = tf.TransformListener()

        self.path_planner = None
        self.feet_names = []
        self.footstep_plan_valid = False
        self.foostep_filename = ""

        self.lift_heights = None
        self.feet = None

        self.footstep_marker_menus = {}
        self.footstep_menu_handles = {}
        self.footstep_markers = {}
        self.footstep_poses = {}
        self.foot_display_offsets = {}
        self.footstep_change_map = []
        
        self.full_footstep_controls = {}
        self.foot_color_map = {}
        self.start_foot_color_map = {}
        
        self.footstep_menu_options = []
        self.footstep_menu_options.append("Toggle Full Controls")
        self.footstep_menu_options.append("Snap To Points")
        self.footstep_menu_options.append("Snap PATH To Points")
        self.footstep_menu_options.append("Add Footstep Before")
        self.footstep_menu_options.append("Add Footstep After")
        self.footstep_menu_options.append("Add Alternate Footstep")
        self.footstep_menu_options.append("Zero Orientation")
        self.footstep_menu_options.append("Delete Footstep")
        self.footstep_menu_options.append("Save Footsteps")
        # self.footstep_menu_options.append("Execute")
        
        self.footstep_sub = rospy.Subscriber("/planner/footsteps_in", MarkerArray, self.footstep_callback)
        self.path_pub = rospy.Publisher("/planner/path", Path, queue_size=1)      
        
        random.seed(rospy.Time.now().secs)

        self.get_footstep_files()

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
                c.g = 0.75-0.5*(float(self.feet_names.index(foot))/float(len(self.feet_names)-1))
                c.a = 0.8
                self.foot_color_map[foot] = c

                c2 = ColorRGBA()
                c2.r = (float(self.feet_names.index(foot))/float(len(self.feet_names)-1))
                c2.b = (float(self.feet_names.index(foot))/float(len(self.feet_names)-1))*.25
                c2.g = 1-(float(self.feet_names.index(foot))/float(len(self.feet_names)-1))
                c2.a = 1
                self.start_foot_color_map[foot] = c2
    

    def set_footstep_poses(self, poses, lift_heights=None, feet=None, transform_poses=True) :
        self.clear_footsteps()
        if self.translate_poses_to_markers(poses, lift_heights, feet, transform_poses) :
            self.create_foot_interactive_markers()
            self.lift_heights = lift_heights
            self.feet = feet


    def footstep_callback(self, data) :
        self.server.setPose(data.marker_name, data.pose)
        self.server.applyChanges()
        
        self.clear_footsteps()
        self.footstep_array = data
        self.create_foot_interactive_markers()


    def translate_poses_to_markers(self, poses, lift_heights, feet, transform_poses=True) :       

        if not poses :
            rospy.logwarn("FootstepControl::translate() -- can't make markers no poses given!")
            return False

        rospy.loginfo(str("FootstepControl::translate() -- Adding " + str(len(poses)) + " footsteps")) 
        
        self.footstep_array = MarkerArray()
        num_feet = len(self.feet_names)
        foot_inc = {}
        full_foot_inc = 0

        for f in self.feet_names :
            foot_inc[f] = 0

        if feet != None :
            if len(feet) != len(poses) :
                rospy.logerr("FootstepControl::translate_poses_to_markers() -- size mismatch between foot lables and poses!!")
                return False

        try :
            if len(feet) > 0 :
                if feet[0].foot == 0: 
                    start_foot = "left"
                else :
                    start_foot = "right"
                start_foot_id = feet[0].foot
        except :
            start_foot = self.path_planner.get_start_foot()
            start_foot_id = self.feet_names.index(start_foot)
    
        for id in range(len(poses)) :
            m = Marker()      
            try :
                if transform_poses :
                    self.tf_listener.waitForTransform(self.frame_id, str("/" + poses[id].header.frame_id), rospy.Time(0), rospy.Duration(5.0))
                    ps = self.tf_listener.transformPose(self.frame_id, poses[id])
                else :
                    ps = poses[id]
                m.header = ps.header
                m.pose = ps.pose
            except :
                rospy.logerr(str("FootstepControl::translate() -- error translating pose from " + poses[id].header.frame_id + " to " + self.frame_id))
                return False
            m.id = id
            # this assumes the feet order in the names is the order assoicated 
            # with the input pose array (modulated by the start foot. probably 
            # a bad assumption in general)
            try :
                if feet[id].foot == 0: 
                    foot_name = "left"
                else :
                    foot_name = "right"
                # m.text = foot_name + "/" + str(foot_inc[foot_name])          
                m.text = foot_name + "/" + str(full_foot_inc)          
                foot_inc[foot_name] += 1
                full_foot_inc += 1          
            except :
                m.text = self.feet_names[(id+start_foot_id)%num_feet] + "/" + str(id/2)            
                rospy.logerr(str("FootstepControl::translate() -- what?"))

            rospy.loginfo(str("FootstepControl::translate() -- Adding foot[" + str(m.text) + "] at (" + str(m.pose.position.x) + ", " + str(m.pose.position.y) + ")_[" + str(m.header.frame_id) + "]"))
            self.footstep_array.markers.append(m)

        return True
        

    def create_foot_interactive_markers(self) :

        if len(self.feet_names)==0 :
            rospy.logerr("FootstepControl::create_foot_interactive_markers() -- no foot names found!")
            return

        first_foot = True
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
            if first_foot :
                foot_color = copy.deepcopy(self.start_foot_color_map[foot_name])
                # foot_color.g = 1
                # foot_color.r = 0.0
                # foot_color.b = 0.0
                first_foot = False
            else :
                foot_color = copy.deepcopy(self.foot_color_map[foot_name])
                
    
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
                       
        self.footstep_plan_valid = True
        self.server.applyChanges()

    def footstep_marker_callback(self, feedback) :
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            handle = feedback.menu_entry_id
            if handle == self.footstep_menu_handles["Toggle Full Controls"] :
                self.toggle_foot_controls(feedback)
                self.create_foot_interactive_markers()
            elif handle == self.footstep_menu_handles["Delete Footstep"] :
                self.delete_footstep(feedback)
            elif handle == self.footstep_menu_handles["Add Footstep Before"] :
                self.add_footstep(feedback, "before")
            elif handle == self.footstep_menu_handles["Add Footstep After"] :
                self.add_footstep(feedback, "after")
            elif handle == self.footstep_menu_handles["Add Alternate Footstep"] :
                self.add_footstep(feedback, "alternate")
            elif handle == self.footstep_menu_handles["Zero Orientation"] :
                self.zero_orientation(feedback.pose, feedback.header.frame_id, feedback.marker_name)
            elif handle == self.footstep_menu_handles["Snap To Points"] :
                self.snap_to_points(feedback.pose, feedback.header.frame_id, feedback.marker_name)
            elif handle == self.footstep_menu_handles["Snap PATH To Points"] :
                self.snap_path_to_points(feedback.header.frame_id)

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(str("FootstepControl::footstep_callback() -- moved foot #" + str(feedback.marker_name)))
            self.footstep_change_map.append(feedback.marker_name)
            self.update_footstep_markers_from_server()
            

    def get_foot_poses(self, markers, filter=False) :
        pose_array = [] 

        d = []
        for id in markers.keys() :
            d.append(int(id))
            d.sort()
            
        for id in d:
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

    def zero_orientation(self, pose_in, frame_id, name) :
        pose = geometry_msgs.msg.PoseStamped()       
        pose.pose = pose_in
        R = kdl.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        rpy = list(R.GetRPY())
        rpy[0] = 0
        rpy[1] = 0
        q =  kdl.Rotation.RPY(rpy[0], rpy[1], rpy[2]).GetQuaternion()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.header.frame_id = frame_id  
        footstep_name = self.footstep_markers[name].description
        self.server.setPose(name, pose.pose)
        self.server.applyChanges()

    def snap_path_to_points(self, frame_id) :
        for f in self.footstep_poses.keys() :
            rospy.sleep(0.4)
            self.snap_to_points(self.footstep_poses[f], frame_id, f)

    def snap_to_points(self, pose_in, frame_id, name) :
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose = pose_in
        pose.header.frame_id = frame_id      

        if frame_id != self.frame_id :
            self.tf_listener.waitForTransform(self.frame_id, frame_id, rospy.Time(0), rospy.Duration(5.0))
            pose = self.tf_listener.transformPose(self.frame_id, pose)

        key = name
        footstep_name = self.footstep_markers[key].description
        foot = footstep_name[0:footstep_name.index("/")]
        new_pose = self.path_planner.snap_footstep_to_points(pose)

        if new_pose :
            rospy.loginfo("FootstepControl::snap_to_points() -- got new footstep pose")
           
            new_pose.pose = self.add_display_offset(new_pose.pose, foot)

            if frame_id != self.frame_id :
                print "transforming back to ", frame_id
                self.tf_listener.waitForTransform(self.frame_id, frame_id, rospy.Time(0), rospy.Duration(5.0))
                new_pose = self.tf_listener.transformPose(frame_id, new_pose)

            new_pose.pose = self.add_display_offset(new_pose.pose, foot)

            self.server.setPose(name, new_pose.pose)
            self.server.applyChanges()
        else :
            rospy.logwarn("FootstepControl::snap_to_points() -- no footstep pose")

    def delete_footstep(self, feedback) :

        key = feedback.marker_name
        footstep_name = self.footstep_markers[key].description
        rospy.loginfo(str("FootstepControl::delete_footstep() -- deleting footstep with key: " + key + " (also known as: " + footstep_name + ")"))

        new_markers = copy.deepcopy(self.footstep_markers)
        del new_markers[key]
        new_lift_heights = self.lift_heights[0:int(key)]+self.lift_heights[int(key):len(self.lift_heights)-1]
        new_feet = copy.deepcopy(self.feet)
        new_feet.pop(int(key))

        new_poses = self.get_foot_poses(new_markers, filter=False)
   
        self.set_footstep_poses(new_poses, new_lift_heights, new_feet, False)
        
    def swap_order(self) :   
        for idx in range(0,len(self.footstep_markers.keys())-1,2) :
            self.swap_footstep(idx,idx+1)
            idx += 2

    def swap_footstep(self, id1, id2) :   
        
        id1_str = str(id1)
        id2_str = str(id2)
        if id1_str not in self.footstep_markers.keys() :
            rospy.logerr(str("FootstepControl::swap_footstep() -- no foostep ID: " + id1_str))
            return
        if id2_str not in self.footstep_markers.keys() :
            rospy.logerr(str("FootstepControl::swap_footstep() -- no foostep ID: " + id2_str))
            return 

        new_markers = copy.deepcopy(self.footstep_markers)
        new_lift_heights = copy.deepcopy(self.lift_heights)
        new_feet = copy.deepcopy(self.feet)      

        # print "--------------"
        # print "original markers: "
        # print new_markers
        swap_marker = new_markers[id1_str]
        new_markers[id1_str] = new_markers[id2_str]
        new_markers[id2_str] = swap_marker
        # print "new markers: "
        # print new_markers

        # print "--------------"
        # print "original lift_heights: "
        # print new_lift_heights
        new_lift_heights = list(new_lift_heights)
        swap_lift_height = new_lift_heights[id1]
        new_lift_heights[id1] = new_lift_heights[id2]
        new_lift_heights[id2] = swap_lift_height
        new_lift_heights = tuple(new_lift_heights)
        # print "new lift_heights: "
        # print new_lift_heights

        # print "--------------"
        # print "original feet: "
        # print new_feet
        swap_foot = new_feet[id1]
        new_feet[id1] = new_feet[id2]
        new_feet[id2] = swap_foot
        # print "new feet: "
        # print new_feet
        # print "--------------"

        new_poses = self.get_foot_poses(new_markers, filter=False)

        self.set_footstep_poses(new_poses, new_lift_heights, new_feet, False)

    def add_footstep(self, feedback, mode) :   

        key = feedback.marker_name
        footstep_name = self.footstep_markers[key].description

        rospy.loginfo(str("FootstepControl::adding footstep() " + mode + " -- key: " + key + " (also known as: " + footstep_name + ")"))

        foot = footstep_name[0:footstep_name.index("/")]
        foot_id = int(footstep_name[footstep_name.index("/")+1:len(footstep_name)])

        new_markers = copy.deepcopy(self.footstep_markers)
        new_poses = self.get_foot_poses(new_markers, filter=False)
        new_lift_heights = copy.deepcopy(self.lift_heights)
        new_feet = copy.deepcopy(self.feet)         

        current_pose = geometry_msgs.msg.PoseStamped()
        current_pose.pose = feedback.pose
        # current_pose.header.frame_id = self.frame_id
        current_pose.header.frame_id = feedback.header.frame_id
        new_pose = copy.deepcopy(current_pose)

        idx = int(feedback.marker_name)
        current_foot = self.feet[idx]
        current_lift_height = self.lift_heights[idx]

        if mode == "before":
           
            new_pose.pose.position.x -= 0.1
            new_pose.pose.position.y -= 0.1
            new_pose.pose = self.remove_display_offset(new_pose.pose, foot)

            lhl = list(new_lift_heights)
            lhl.insert(idx,new_lift_heights[idx])
            new_lift_heights = tuple(lhl)
            new_feet.insert(idx,new_feet[idx])
            new_poses.insert(idx,new_pose)

        elif mode == "after" :

            new_pose.pose.position.x += 0.1
            new_pose.pose.position.y += 0.1
            new_pose.pose = self.remove_display_offset(new_pose.pose, foot)

            lhl = list(new_lift_heights)
            lhl.insert(idx+1,new_lift_heights[idx])
            new_lift_heights = tuple(lhl)
            
            new_feet.insert(idx+1,new_feet[idx])
            new_poses.insert(idx+1,new_pose)
        
        elif mode == "alternate" :

            if foot == "left" :
                new_pose.pose.position.y -= 0.15
            else :
                new_pose.pose.position.y += 0.15

            new_pose.pose = self.remove_display_offset(new_pose.pose, foot)

            lhl = list(new_lift_heights)
            lhl.insert(idx+1,new_lift_heights[idx])
            new_lift_heights = tuple(lhl)
            
            other_foot = copy.deepcopy(new_feet[idx])
            other_foot.foot = 1-other_foot.foot
            new_feet.insert(idx+1,other_foot)
            new_poses.insert(idx+1,new_pose)


        else :
            rospy.logerr(str("FootstepControl::add_footstep() -- unknown mode: " + False))


        self.set_footstep_poses(new_poses, new_lift_heights, new_feet, False)



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
        rospy.loginfo(str("FootstepControl::update_footstep_markers_from_server()"))
        for m in self.footstep_markers.keys() :
            self.footstep_markers[m].pose = self.server.get(m).pose

    def execute_footstep_path(self) :
        rospy.loginfo(str("FootstepControl::execute_footstep_path() -- executing footstep path: " + str(self.footstep_plan_valid)))
        self.update_footstep_markers_from_server()
        if len(self.footstep_markers.keys())>0 and self.footstep_plan_valid :
            step_poses = self.get_foot_poses(self.footstep_markers, filter=False)
            for id in range(len(step_poses)) :
                if self.frame_id != step_poses[id].header.frame_id :
                    self.tf_listener.waitForTransform(self.frame_id, str("/" + step_poses[id].header.frame_id), rospy.Time(0), rospy.Duration(5.0))
                    step_poses[id] = self.tf_listener.transformPose(self.frame_id, step_poses[id])
            ret = self.path_planner.execute_navigation_plan(step_poses, self.lift_heights, self.feet)
            rospy.loginfo(str("FootstepControl::execute_footstep_path() -- returned: " + str(ret)))            
            self.footstep_plan_valid = False

    def set_footstep_filename(self, filename) :
        self.footstep_filename = filename


    def save_footsteps(self) :

        import datetime
        import rospkg

        d=datetime.datetime.now()

        rp = rospkg.RosPack()
        path = rp.get_path("nasa_robot_teleop")

        if self.footstep_filename == "" :
            filename =  path + "/store/footpaths/" + d.strftime("%d-%m-%Y-%H-%M-%S") + str(".fsp")
        else :
            filename = path + "/store/footpaths/" + self.footstep_filename

        self.update_footstep_markers_from_server()
        step_poses = self.get_foot_poses(self.footstep_markers, filter=False)

        for idx in range(len(step_poses)) :
            step_poses[idx].header.stamp = rospy.Time(0)
            self.tf_listener.waitForTransform("nav_goal", step_poses[idx].header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            step_poses[idx] = self.tf_listener.transformPose("nav_goal", step_poses[idx])          

        storage = FootstepStorage()
        storage.step_poses = step_poses
        storage.feet = self.feet
        storage.lift_heights = self.lift_heights

        rospy.logwarn("FootstepControl::save_footsteps() -- saved to: " + filename)
        pickle.dump(storage, open( filename, "wb"))

        self.filename = filename

        self.get_footstep_files()
        

    def load_footsteps_from_file(self, filename) :

        print "load_footsteps_from_file() -- test 1"
        import rospkg
        rp = rospkg.RosPack()
        print "load_footsteps_from_file() -- test 2"
        path = rp.get_path("nasa_robot_teleop")
        print "load_footsteps_from_file() -- test 3"

        recalled = pickle.load( open( filename, "rb") )
        print "load_footsteps_from_file() -- test 4"

        step_poses = []
        print "load_footsteps_from_file() -- test 5"
        for idx in range(len(recalled.step_poses)) :
            # if not idx in recalled.step_poses.keys() :
            #     continue 
            step = recalled.step_poses[idx]
            step.header.stamp = rospy.Time(0)
            # self.tf_listener.waitForTransform("nav_goal", step.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            # new_step = self.tf_listener.transformPose("nav_goal", step)     
            step_poses.append(step)
            print "load_footsteps_from_file() -- test 6"
    
        print "load_footsteps_from_file() -- test 7"
        self.set_footstep_poses(step_poses, recalled.lift_heights, recalled.feet, False)
        print "load_footsteps_from_file() -- test 8"


    def get_footstep_files(self) :
        
        import rospkg, os, glob

        rp = rospkg.RosPack()
        path = rp.get_path("nasa_robot_teleop") + "/store/footpaths/"
        os.chdir(path)

        self.footpaths = []
        for ffile in glob.glob("*.fsp") :
            self.footpaths.append(ffile)

        return self.footpaths


if __name__=="__main__":

    rospy.init_node("FootstepControl")

    server = InteractiveMarkerServer(str("atlas_interactive_marker_server"))
    tf_listener = tf.TransformListener()

    fc = FootstepControl("atlas", server, "global", tf_listener)

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        r.sleep()
