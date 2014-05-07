#! /usr/bin/env python

import rospy
import roslib

import threading
import geometry_msgs
import PyKDL as kdl

from nasa_robot_teleop.kdl_posemath import *

class PoseUpdateThread(threading.Thread) :
    def __init__(self, name, root_frame, control_frame, tf_listener, offset_pose) :
        super(PoseUpdateThread,self).__init__()
        self.mutex = threading.Lock()
        self.name = name
        self.pose_data = geometry_msgs.msg.PoseStamped()
        self.tf_listener = tf_listener
        self.control_frame = control_frame
        self.root_frame = root_frame
        self.is_valid = False
        self.offset_pose = offset_pose
        self.T_offset = kdl.Frame()
        if offset_pose != None :
            self.T_offset = fromMsg(self.offset_pose)

    def run(self) :
        while True :
            self.mutex.acquire()
            try :
                try :
                    self.tf_listener.waitForTransform(self.control_frame,self.root_frame, rospy.Time(0), rospy.Duration(3.0))
                    (trans, rot) = self.tf_listener.lookupTransform(self.root_frame, self.control_frame, rospy.Time(0))
                    T = fromMsg(toPose(trans, rot))
                    self.pose_data = toMsg(T*self.T_offset)
                    self.is_valid = True
                except :
                    rospy.logdebug("PoseUpdateThread::run() -- could not update thread")
            finally :
                self.mutex.release()

            rospy.sleep(0.1)

    def get_pose_data(self) :
        self.is_valid = False
        return self.pose_data
