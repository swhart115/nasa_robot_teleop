#! /usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

import threading
import PyKDL as kdl

from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.urdf_parser_py import *
from nasa_robot_teleop.pose_update_thread import *

class EndEffectorHelper :

    def __init__(self, robot, name, root_frame, tf_listener) :

        self.link_meshes = {}
        self.link_origins = {}
        self.offset_pose_data = {}
        self.offset_update_thread = {}
        self.links = []

        self.control_frame = ""
        self.control_pose = Pose

        self.current_marker_array = None
        self.robot_name = robot
        self.name = name
        self.root_frame = root_frame
        self.tf_listener = tf_listener

    def add_link(self, link, mesh, origin) :
        self.link_meshes[link] = mesh
        self.link_origins[link] = origin
        self.links.append(link)

    def populate_data(self, links, urdf) :

        for link in links :
            if not link in urdf.link_map :
                print "EndEffectorHelper::populate_data() -- link: ", link, " not found in URDF model"
                return

            model_link = urdf.link_map[link]
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
                            self.add_link(link, mesh, p)

        self.start_offset_update_thread()

    def has_link(self, link) :
        return link in self.links

    def get_links(self) :
        return self.links

    def get_link_data(self, link) :
        if not self.has_link(link) : return False
        if not self.offset_update_thread[link].get_pose_data() : return False
        return (self.link_meshes[link], self.offset_update_thread[link].get_pose_data())

    def set_control_frame(self, control_pose, control_mesh) :
        self.control_pose = control_pose
        self.control_mesh = control_mesh

    def start_offset_update_thread(self) :
        print "EndEffectorHelper::start_offset_update_thread() -- starting offset update thread for end effector from root: ", self.root_frame
        for link in self.links :
            self.offset_pose_data[link] = PoseStamped()
            try :
                self.offset_update_thread[link] = PoseUpdateThread(link, self.root_frame, link, self.tf_listener, self.link_origins[link])
                self.offset_update_thread[link].start()
            except :
                rospy.logerr("EndEffectorHelper::start_offset_update_thread() -- unable to start end effector link offset update thread")

    def get_link_offset(self, link) :
        return self.offset_pose_data[link]

    def get_root_frame(self) :
        return self.root_frame

    def get_current_position_marker_array(self, offset=None, root="", scale=1, color=(0,1,0,1), idx=0) :

        markers = MarkerArray()
        if root=="": root = self.root_frame

        for link in self.get_links() :
            if self.get_link_data(link) :

                (mesh, pose) = self.get_link_data(link)

                marker = Marker()

                if offset==None :
                    marker.pose = pose
                else :
                    marker.pose = toMsg(fromMsg(offset)*fromMsg(pose))

                marker.header.frame_id = root
                marker.header.stamp = rospy.get_rostime()
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
                markers.markers.append(marker)
                idx += 1

        self.current_marker_array = markers
        return markers
