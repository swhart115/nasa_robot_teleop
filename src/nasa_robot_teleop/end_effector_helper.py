#! /usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

import threading
import PyKDL as kdl

from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.urdf_helper import *
from nasa_robot_teleop.urdf_parser_py import *
from nasa_robot_teleop.pose_update_thread import *

class EndEffectorHelper :

    def __init__(self, robot, name, root_frame, tf_listener) :

        self.link_meshes = {}
        self.link_origins = {}
        self.offset_pose_data = {}
        self.offset_update_thread = {}
        self.links = []
        self.urdf = None
        self.srdf = None

        self.control_frame = ""
        self.control_pose = Pose

        self.current_marker_array = None
        self.robot_name = robot
        self.name = name
        self.root_frame = root_frame
        self.tf_listener = tf_listener

        self.pose_marker_arrays = {}

    def add_link(self, link, mesh, origin) :
        self.link_meshes[link] = mesh
        self.link_origins[link] = origin
        self.links.append(link)

    def set_urdf(self, urdf) :
        self.urdf = urdf

    def set_srdf(self, srdf) :
        self.srdf = srdf

    def get_markers_for_pose(self, pose) :

        markers = MarkerArray()

        # compute current position marker array
        if pose == "current" :
            markers = self.get_current_position_marker_array(root=self.root_name)
            return markers

        # look up stored pose or compute it if it doesnt exist
        if not pose in self.pose_marker_arrays.keys() :
            if not self.srdf == None :
                poses = self.srdf.get_group_state_list(self.name)
                poses.sort()
                N = len(poses)
                color = (0,poses.index(pose)/(N-1,0,1))
                print "color for pose: ", pose, ": ", color
                if not pose in poses :
                    rospy.logerr("EndEffectorHelper::get_markers_for_pose() -- no pose available!!")
                else :
                    jpos = self.srdf.get_group_state(self.name, pose).to_joint_state_msg()
                    self.pose_marker_arrays[pose] = self.get_marker_array_from_joint_position(jpos=jpos, root=self.root_frame, color=color)
                    markers = self.pose_marker_arrays[pose]
            else :
                rospy.logerr("EndEffectorHelper::get_markers_for_pose() -- no SRDF set!!")
        else :
            markers = self.pose_marker_arrays[pose]

        return markers


    def populate_data(self, links, urdf, srdf) :

        if self.srdf == None : self.set_srdf(srdf)
        if self.urdf == None : self.set_urdf(urdf)

        if not self.srdf == None :
            poses = self.srdf.get_group_state_list(self.name)
            N = len(poses)
            for p in poses :
                c = 0.5+(poses.index(p)/(N-1.0))/2.0
                color = (0,c,0.67+c/3.0,1)
                jpos = self.srdf.get_group_state(self.name, p).to_joint_state_msg()
                self.pose_marker_arrays[p] = self.get_marker_array_from_joint_position(jpos=jpos, root=self.root_frame, color=color)

        for link in links :
            if not link in self.urdf.link_map :
                print "EndEffectorHelper::populate_data() -- link: ", link, " not found in URDF model"
                return

            model_link = self.urdf.link_map[link]
            if model_link :
                if model_link.visual  :
                    if model_link.visual.geometry  :
                        try :
                            if model_link.visual.geometry.filename  :
                                mesh = model_link.visual.geometry.filename
                        except :
                            mesh = None 
                        p = geometry_msgs.msg.Pose()
                        q = (kdl.Rotation.RPY(model_link.visual.origin.rpy[0],model_link.visual.origin.rpy[1],model_link.visual.origin.rpy[2])).GetQuaternion()
                        p.position.x = model_link.visual.origin.xyz[0]
                        p.position.y = model_link.visual.origin.xyz[1]
                        p.position.z = model_link.visual.origin.xyz[2]
                        p.orientation.x = q[0]
                        p.orientation.y = q[1]
                        p.orientation.z = q[2]
                        p.orientation.w = q[3]
                        self.add_link(link, mesh, p)

        self.start_offset_update_thread()

    def has_link(self, link) :
        return link in self.links

    def get_links(self) :
        return self.links

    def get_link_data(self, link) :
        if not self.has_link(link) : return (False, geometry_msgs.msg.Pose())
        if not self.offset_update_thread[link].get_pose_data() : return (False, self.offset_update_thread[link].get_pose_data())
        return (self.link_meshes[link], self.offset_update_thread[link].get_pose_data())

    def set_control_frame(self, control_pose, control_mesh) :
        self.control_pose = control_pose
        self.control_mesh = control_mesh

    def start_offset_update_thread(self) :
        print "EndEffectorHelper::start_offset_update_thread() -- starting offset update thread for end effector from root: ", self.root_frame
        for link in self.links :
            self.offset_pose_data[link] = PoseStamped()
            # try :
            self.offset_update_thread[link] = PoseUpdateThread(link, self.root_frame, link, self.tf_listener, self.link_origins[link])
            self.offset_update_thread[link].start()
            # except :
            #     rospy.logerr("EndEffectorHelper::start_offset_update_thread() -- unable to start end effector link offset update thread")

    def stop_offset_update_thread(self) :
        print "EndEffectorHelper::stop_offset_update_thread() -- stopping offset update thread for end effector from root: ", self.root_frame
        for link in self.links :
            try :
                self.offset_update_thread[link].stop()
            except :
                rospy.logerr("EndEffectorHelper::stop_offset_update_thread() -- unable to stop end effector link offset update thread")

    def get_link_offset(self, link) :
        return self.offset_pose_data[link]

    def get_root_frame(self) :
        return self.root_frame

    def get_current_position_marker(self, link, offset=None, root="", scale=1, color=(0,1,0,1), idx=0):

        (mesh, pose) = self.get_link_data(link)
        marker = Marker()

        s = [scale, scale, scale]
        if offset==None :
            marker.pose = pose
        else :
            marker.pose = toMsg(fromMsg(offset)*fromMsg(pose))

        marker.header.frame_id = root
        marker.header.stamp = rospy.get_rostime()
        marker.ns = self.robot_name
        if mesh :
            marker.mesh_resource = mesh
            marker.type = Marker.MESH_RESOURCE
            # need to set scale....
        else :
            scale = 0.001
            marker.type = Marker.SPHERE
            print "setting SPHERE for link: ", link
            return None
        marker.action = Marker.MODIFY
        marker.scale.x = s[0]
        marker.scale.y = s[1]
        marker.scale.z = s[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = link
        marker.id = idx
        marker.mesh_use_embedded_materials = True
        return marker


    def create_marker_for_link(self, link, T, scale=1, color=(0,1,0,1), idx=0) :
        marker = Marker()

        if not link in self.urdf.link_map :
            print "EndEffectorHelper::create_marker_for_link() -- link: ", link, " not found in URDF model"
            return marker

        model_link = self.urdf.link_map[link]
        if model_link :
            if model_link.visual  :
                if model_link.visual.geometry  :
                    if model_link.visual.geometry.filename  :
                        mesh = model_link.visual.geometry.filename
                        p = geometry_msgs.msg.Pose()
                        q = (kdl.Rotation.RPY(model_link.visual.origin.rpy[0],model_link.visual.origin.rpy[1],model_link.visual.origin.rpy[2])).GetQuaternion()
                        p.position.x = model_link.visual.origin.xyz[0]
                        p.position.y = model_link.visual.origin.xyz[1]
                        p.position.z = model_link.visual.origin.xyz[2]
                        p.orientation.x = q[0]
                        p.orientation.y = q[1]
                        p.orientation.z = q[2]
                        p.orientation.w = q[3]

                        marker.pose = toMsg(T*fromMsg(p))

                        marker.header.frame_id = self.root_frame
                        marker.ns = self.robot_name
                        marker.mesh_resource = mesh
                        marker.type = Marker.MESH_RESOURCE
                        marker.action = Marker.MODIFY
                        marker.scale.x = scale
                        marker.scale.y = scale
                        marker.scale.z = scale
                        marker.color.r = color[0]
                        marker.color.g = color[1]
                        marker.color.b = color[2]
                        marker.color.a = color[3]
                        marker.text = link
                        marker.id = idx
                        marker.mesh_use_embedded_materials = True

        return marker



    def get_current_position_marker_array(self, offset=None, root="", scale=1, color=(0,1,0,1), idx=0) :

        markers = MarkerArray()
        if root=="": root = self.root_frame

        for link in self.get_links() :
            if self.get_link_data(link) :
                marker = self.get_current_position_marker(link, offset, root, scale, color, idx)
                if not marker == None :
                    markers.markers.append(marker)
                idx += 1

        self.current_marker_array = markers
        return markers

    def get_marker_array_from_joint_position(self, jpos, offset=None, root="", scale=1, color=(0,1,1,1), idx=0) :

        # print "EndEffectorHelper::get_marker_array_from_joint_position()"

        markers = MarkerArray()
        if root=="": root = self.root_frame

        link_list = []
        link_joints = {}
        T_joint = {}
        T_link = {}

        for j in jpos.name :
            link = self.urdf.link_map[self.urdf.joint_map[j].child].name
            if not link in link_joints.keys(): link_joints[link] = j
            if not link in link_list : link_list.append(link)

            model_joint = self.urdf.joint_map[j]
            joint_val = jpos.position[jpos.name.index(j)]
            T_joint[j] = get_joint_rotation(model_joint.axis, joint_val)

        if not root in link_list :
            link_list.append(root)
            link_joints[root] = get_link_joint(root, self.urdf)
        # print link_list

        def get_transform_to_link(root_frame, link, joint, T_link) :

            # print "Getting transform of link: ", link, " with joint: ", joint
            # print "root_frame:", root_frame
            # already is in list
            if link in T_link.keys() :
                # print "Already found Transform for link: ", link
                return T_link[link]

            # check first link
            # if link == root_frame :
            #     print "At base Transform: ", root_frame
            #     return kdl.Frame()

            if not self.urdf.joint_map[joint].type == "fixed":
                if not joint in jpos.name :
                    # print "\n------\nFound joint ", joint, " for link: ", link, " but it is not in the jpos list, looking up frame and exiting.."
                    self.tf_listener.waitForTransform(root_frame, link, rospy.Time(0), rospy.Duration(5.0))
                    (trans, rot) = self.tf_listener.lookupTransform(root_frame, link, rospy.Time(0))
                    rot = normalize_vector(rot)
                    T = fromMsg(toPose(trans,rot)).Inverse()
                    # print T
                    return kdl.Frame()

            # call recursive function to parent, then add link transform, then add joint transform
            parent = get_parent_link(link, self.urdf)
            parent_joint = get_link_joint(parent, self.urdf)
            #rint "\tNeed to go to parent link: ", parent, " with joint: ", parent_joint
            T = get_transform_to_link(root_frame, parent, parent_joint, T_link)

            model_joint = self.urdf.joint_map[joint]
            T_kin = fromMsg(joint_origin_to_pose(model_joint))
            # print "Link: ", link, " -- computing tf for joint: ", joint, "between ", self.urdf.joint_map[joint].parent , "<-->", self.urdf.joint_map[joint].child
            # # print T_kin

            if joint in T_joint.keys() :
                T_link[link] = T*T_kin*T_joint[joint]
            else :
                T_link[link] = T*T_kin
            return T_link[link]

        idx = 0
        for link in link_list :
            T_link[link] = get_transform_to_link(root, link, link_joints[link], T_link)
            # print "\n---------\n----------Found Transform to link: ", link
            # print T_link[link]

            if link_has_mesh(self.urdf.link_map[link]) :
                markers.markers.append(self.create_marker_for_link(link, T_link[link], scale=scale, color=color, idx=idx))
                idx += 1

        return markers

    