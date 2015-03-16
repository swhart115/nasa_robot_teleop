#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("nasa_robot_teleop")

import math

# ros messages
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path


    
if __name__=="__main__":

    rospy.init_node("DummyFootstepPublisher")

    footstep_pub = rospy.Publisher("/planner/footsteps_in", MarkerArray)
    path_pub = rospy.Publisher("/planner/path", Path)
        
    footsteps = MarkerArray()

    foot_width = 0.125
    foot_dist = 0.25

    angle = 0.1

    path = Path()
    path.header.frame_id = "/ground"
    path.header.stamp = rospy.Time.now()

    last_point = PoseStamped()
    for id in range(10) :

        footstep = Marker()
        footstep.header.stamp = rospy.Time.now()
        footstep.header.seq = id
        footstep.header.frame_id = "/ground"
        footstep.id = id
        footstep.action = 0
        p = Pose()
        p.position.x = foot_dist*id
        
        if id%2 == 0 :
            p.position.y = -foot_width
            footstep.text = "left/" + str(id/2)
            last_point = p
        else :
            p.position.y =  foot_width
            footstep.text = "right/" + str(id/2)

            pp = PoseStamped()
            pp.header.frame_id = "/ground"
            pp.header.seq = id/2
            pp.header.stamp = rospy.Time.now()
            pp.pose.position.x = (p.position.x + last_point.position.x)/2.0
            pp.pose.position.y = (p.position.y + last_point.position.y)/2.0
            pp.pose.position.z = (p.position.z + last_point.position.z)/2.0


            path.poses.append(pp)
        footstep.ns = "footstep"
        p.position.z = 0
        p.orientation.w = 1.0

        footstep.pose = p
        footsteps.markers.append(footstep)

    print footsteps

    for i in range(2) :
        footstep_pub.publish(footsteps)
        path_pub.publish(path)
        rospy.sleep(1)