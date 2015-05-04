#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

roslib.load_manifest("auto_walker")

import math
import random
import tf
import pickle

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

        self.footstep_marker_menus = {}
        self.footstep_menu_handles = {}

        self.footstep_markers = {}
        self.full_footstep_controls = {}
        self.footstep_poses = {}
       
        self.footstep_menu_options = []
        self.footstep_menu_options.append("Toggle Full Controls")
        self.footstep_menu_options.append("Save Footsteps")
        # self.footstep_menu_options.append("Execute")
        
        self.footstep_plan_valid = False

        self.footstep_sub = rospy.Subscriber("/planner/footsteps_in", MarkerArray, self.footstep_callback)
        self.path_pub = rospy.Publisher("/planner/path", Path, queue_size=1)      
        
        self.foot_display_offsets = {}
        self.footstep_change_map = []
        self.foot_color_map = {}
        self.feet_names = []

        self.lift_heights = None
        self.feet = None

        self.foostep_filename = ""

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
                c.a = 1.0
                self.foot_color_map[foot] = c
    

    def set_footstep_poses(self, poses, lift_heights=None, feet=None) :
        self.clear_footsteps()
        if self.translate_poses_to_markers(poses, lift_heights, feet) :
            self.create_foot_interactive_markers()
            self.lift_heights = lift_heights
            self.feet = feet


    def footstep_callback(self, data) :
        self.clear_footsteps()
        self.footstep_array = data
        self.create_foot_interactive_markers()


    def translate_poses_to_markers(self, poses, lift_heights, feet) :       

        if not poses :
            rospy.logwarn("FootstepControl::translate() -- can't make markers no poses given!")
            return False

        rospy.loginfo(str("FootstepControl::translate() -- Adding " + str(len(poses)) + " footsteps")) 
        
        self.footstep_array = MarkerArray()
        num_feet = len(self.feet_names)
        
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
                self.tf_listener.waitForTransform(self.frame_id, str("/" + poses[id].header.frame_id), rospy.Time(0), rospy.Duration(5.0))
                # self.tf_listener.waitForTransform(self.frame_id, str(poses[id].header.frame_id), rospy.Time(0), rospy.Duration(3.0))
                ps = self.tf_listener.transformPose(self.frame_id, poses[id])
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
                m.text = foot_name + "/" + str(id/2)            
            except :
                m.text = self.feet_names[(id+start_foot_id)%num_feet] + "/" + str(id/2)            

            rospy.loginfo(str("FootstepControl::translate() -- Adding foot[" + str(m.text) + "] at (" + str(m.pose.position.x) + ", " + str(m.pose.position.y) + ")_[" + str(m.header.frame_id) + "]"))
            self.footstep_array.markers.append(m)

        return True
        

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
            elif handle == self.footstep_menu_handles["Save Footsteps"] :
                self.save_footsteps()

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(str("FootstepControl::footstep_callback() -- moved foot #" + str(feedback.marker_name)))
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
        rospy.loginfo(str("FootstepControl::update_footstep_markers_from_server()"))
         
        for m in self.footstep_markers.keys() :
            self.footstep_markers[m].pose = self.server.get(m).pose

    def execute_footstep_path(self) :
        rospy.loginfo(str("FootstepControl::execute_footstep_path() -- executing footstep path: " + str(self.footstep_plan_valid)))
        self.update_footstep_markers_from_server()
        if len(self.footstep_markers)>0 and self.footstep_plan_valid :
            step_poses = self.get_foot_poses(self.footstep_markers, filter=False)
            # print "==== FINAL STEP POSES ==="
            # print step_poses
            # print "======="
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

        print step_poses
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

        import rospkg
        rp = rospkg.RosPack()
        path = rp.get_path("nasa_robot_teleop")

        recalled = pickle.load( open( filename, "rb") )

        step_poses = []
        for idx in range(len(recalled.step_poses)) :
            step = recalled.step_poses[idx]
            step.header.stamp = rospy.Time(0)
            # self.tf_listener.waitForTransform("nav_goal", step.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            # new_step = self.tf_listener.transformPose("nav_goal", step)     
            step_poses.append(step)
            
        self.set_footstep_poses(step_poses, recalled.lift_heights, recalled.feet)
        

    def get_footstep_files(self) :
        
        import rospkg, os, glob

        rp = rospkg.RosPack()
        path = rp.get_path("nasa_robot_teleop") + "/store/footpaths/"
        os.chdir(path)

        self.footpaths = []
        for ffile in glob.glob("*.fsp") :
            self.footpaths.append(ffile)

        return self.footpaths

    # def load_in_stairs(self, filename) :

    #     stair_poses_raw = []
    #     stair_pose_frame_raw = "global"

    #     feet = []
    #     pose = geometry_msgs.msg.Pose() 
    #     foot = Foot()
    #     ps = geometry_msgs.msg.PoseStamped() 

    #     # left 0
    #     pose.position.x = 4.47881603392
    #     pose.position.y = 2.81686902029
    #     pose.position.z = 0.44295186449
    #     pose.orientation.x = 0
    #     pose.orientation.y = 0
    #     pose.orientation.z = 0.562231245389
    #     pose.orientation.w = 0.826980064275
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 0
    #     feet.append(copy.deepcopy(foot))

    #     # right 0
    #     pose.position.x = 4.63931035982
    #     pose.position.y = 2.59922361407
    #     pose.position.z = 0.441650181786
    #     pose.orientation.x = 0
    #     pose.orientation.y = 0
    #     pose.orientation.z = 0.0476384133447
    #     pose.orientation.w = 0.998864646273
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 1
    #     feet.append(copy.deepcopy(foot))
        
    #     # left 1
    #     pose.position.x = 4.73656129849
    #     pose.position.y = 3.0028874885
    #     pose.position.z = 0.681394671407
    #     pose.orientation.x = 0
    #     pose.orientation.y = 0
    #     pose.orientation.z = 0.556873673663
    #     pose.orientation.w = 0.830597201765
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 0
    #     feet.append(copy.deepcopy(foot))
        
    #     # right 1
    #     pose.position.x = 4.89289236067
    #     pose.position.y = 2.77921843691
    #     pose.position.z = 0.67948231753
    #     pose.orientation.x = 0
    #     pose.orientation.y = 0
    #     pose.orientation.z = 0.0636715264746
    #     pose.orientation.w = 0.997970909755
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 1
    #     feet.append(copy.deepcopy(foot))
        
    #     # left 2
    #     pose.position.x = 4.99311733438
    #     pose.position.y = 3.1734027859
    #     pose.position.z = 0.905397747432
    #     pose.orientation.x = -0
    #     pose.orientation.y = -0
    #     pose.orientation.z = 0.543764702186
    #     pose.orientation.w = 0.8392377188
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 0
    #     feet.append(copy.deepcopy(foot))
        
    #     # right 2
    #     pose.position.x = 5.13633060461
    #     pose.position.y = 2.95412588184
    #     pose.position.z = 0.937160230172
    #     pose.orientation.x = 0
    #     pose.orientation.y = 0
    #     pose.orientation.z = 0.0583485740294
    #     pose.orientation.w = 0.998296270607
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 1
    #     feet.append(copy.deepcopy(foot))
        
    #     # left 3
    #     pose.position.x = 5.25616073608
    #     pose.position.y = 3.3876311779
    #     pose.position.z = 1.15746102294
    #     pose.orientation.x = 0.0
    #     pose.orientation.y = 0.0
    #     pose.orientation.z = 0.313186852916
    #     pose.orientation.w = 0.949691526318
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 0
    #     feet.append(copy.deepcopy(foot))
        
    #     # right 3
    #     pose.position.x = 5.37831258774
    #     pose.position.y = 3.22746706009
    #     pose.position.z = 1.15021762937
    #     pose.orientation.x = 0.0
    #     pose.orientation.y = 0.0
    #     pose.orientation.z = 0.313070896071
    #     pose.orientation.w = 0.949729758423
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 1
    #     feet.append(copy.deepcopy(foot))
        
    #     # left 4
    #     pose.position.x = 5.35390138626
    #     pose.position.y = 3.44903278351
    #     pose.position.z = 1.16442689866
    #     pose.orientation.x = 0.0
    #     pose.orientation.y = 0.0
    #     pose.orientation.z = 0.312710552469
    #     pose.orientation.w = 0.949848467059
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 0
    #     feet.append(copy.deepcopy(foot))
        
    #     # right 4
    #     pose.position.x = 5.47334909439
    #     pose.position.y = 3.29888057709
    #     pose.position.z = 1.15543255844
    #     pose.orientation.x = 0.0
    #     pose.orientation.y = 0.0
    #     pose.orientation.z = 0.31136793018
    #     pose.orientation.w = 0.950289435938
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 1
    #     feet.append(copy.deepcopy(foot))
        
    #     # left 5
    #     pose.position.x = 5.51736450195
    #     pose.position.y = 3.56981611252
    #     pose.position.z = 1.154797648
    #     pose.orientation.x = 0.0
    #     pose.orientation.y = 0.0
    #     pose.orientation.z = 0.307339514996
    #     pose.orientation.w = 0.951599927765
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 0
    #     feet.append(copy.deepcopy(foot))
        
    #     # right 5
    #     pose.position.x = 5.63400173187
    #     pose.position.y = 3.41920757294
    #     pose.position.z = 1.15934512476
    #     pose.orientation.x = 0.0
    #     pose.orientation.y = 0.0
    #     pose.orientation.z = 0.307339514996
    #     pose.orientation.w = 0.951599927765
    #     ps.header.frame_id = stair_pose_frame_raw
    #     ps.header.stamp = rospy.Time(0)
    #     ps.pose = copy.deepcopy(pose)
    #     stair_poses_raw.append(copy.deepcopy(ps))
    #     foot.foot = 1
    #     feet.append(copy.deepcopy(foot))
        
    #     print stair_poses_raw        

    #     for idx in range(len(stair_poses_raw)) :
    #         stair_poses_raw[idx].header.stamp = rospy.Time(0)
    #         self.tf_listener.waitForTransform("nav_goal", stair_poses_raw[idx].header.frame_id, rospy.Time(0), rospy.Duration(3.0))
    #         stair_poses_raw[idx] = self.tf_listener.transformPose("nav_goal", stair_poses_raw[idx])          

    #     rospy.logwarn("FootstepControl::save_footsteps() -- saved to: " + filename)

    #     storage = FootstepStorage()
    #     storage.step_poses = stair_poses_raw
    #     storage.feet = feet
    #     storage.lift_heights = [0]*len(feet)


    #     pickle.dump(storage, open( filename, "wb"))




if __name__=="__main__":

    rospy.init_node("FootstepControl")

    server = InteractiveMarkerServer(str("atlas_interactive_marker_server"))
    tf_listener = tf.TransformListener()

    fc = FootstepControl("atlas", server, "global", tf_listener)

    # fc.load_in_stairs("new_stairs.fsp")

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        r.sleep()
