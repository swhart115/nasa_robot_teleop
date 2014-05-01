#! /usr/bin/env python

import rospy
import roslib
import tf

import threading
import geometry_msgs
import PyKDL as kdl

from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.urdf_parser_py import *
from nasa_robot_teleop.pose_update_thread import *

class EndEffectorHelper :

    def __init__(self, root_frame, tf_listener) :

        self.link_meshes = {}
        self.link_origins = {}
        self.offset_pose_data = {}
        self.offset_update_thread = {}
        self.links = []
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

    def start_offset_update_thread(self) :
        for link in self.links :
            self.offset_pose_data[link] = geometry_msgs.msg.PoseStamped()
            try :
                self.offset_update_thread[link] = PoseUpdateThread(link, self.root_frame, link, self.tf_listener, self.link_origins[link])
                self.offset_update_thread[link].start()
            except :
                rospy.logerr("EndEffectorHelper::start_offset_update_thread() -- unable to start end effector link offset update thread")

    def get_link_offset(self, link) :
        return self.offset_pose_data[link]
