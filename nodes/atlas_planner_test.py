#! /usr/bin/env python

import random

import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg

from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.tolerances import *

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *

class AtlasDummyService(object) :

    def __init__(self) :

        self.get_config_service = rospy.Service('/atlas_planner/get_config', GetPlanningServiceConfiguration, self.handle_get_config)
        self.planner_service = rospy.Service('/atlas_planner/plan_command', PlanCommand, self.handle_plan_command)
        self.execute_service = rospy.Service('/atlas_planner/execute_command', ExecuteCommand, self.handle_execute_command)

    def handle_get_config(self, req):
        print "Hello"
        response = GetPlanningServiceConfigurationResponse()

        # N = 3
        # neck = PlanGroupConfiguration()
        # neck.joint_map = JointNameMap()
        # neck.joint_map.names = []
        # neck.joint_map.ids = []
        # for i in range(N) :
        #     neck.joint_map.names.append(str("r2/neck/joint" + str(i)))
        #     neck.joint_map.ids.append(i)
        # neck.group_id = 0
        # neck.group_name = "left_arm"
        # neck.joint_mask = JointMask()
        # neck.joint_mask.mask = [True]*N
        # neck.control_frame = "r2/left_palm"
        # neck.planning_frame = "/world"

        N = 7 

        left_arm_group = PlanGroupConfiguration()
        left_arm_group.joint_map = JointNameMap()
        left_arm_group.joint_map.names = []
        left_arm_group.joint_map.ids = []
        for i in range(N) :
            left_arm_group.joint_map.names.append(str("r2/left_arm/joint" + str(i)))
            left_arm_group.joint_map.ids.append(i)
        left_arm_group.group_id = 0
        left_arm_group.group_name = "left_arm"
        left_arm_group.joint_mask = JointMask()
        left_arm_group.joint_mask.mask = [True]*7
        left_arm_group.control_frame = "r2/left_palm"
        left_arm_group.planning_frame = "/world"

        right_arm_group = PlanGroupConfiguration()
        right_arm_group.joint_map = JointNameMap()
        right_arm_group.joint_map.names = []
        right_arm_group.joint_map.ids = []
        for i in range(N) :
            right_arm_group.joint_map.names.append(str("r2/right_arm/joint" + str(i)))
            right_arm_group.joint_map.ids.append(i)
        right_arm_group.group_id = 0
        right_arm_group.group_name = "right_arm"
        right_arm_group.joint_mask = JointMask()
        right_arm_group.joint_mask.mask = [True]*7
        right_arm_group.control_frame = "r2/right_palm"
        right_arm_group.planning_frame = "/world"


        left_hand_group = PlanGroupConfiguration()
        left_hand_group.joint_map = JointNameMap()
        left_hand_group.joint_map.names = []
        left_hand_group.joint_map.ids = []
        left_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint0")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint1")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint2")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint3")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/little/joint0")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/little/joint1")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/little/joint2")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint0")  
        left_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint1")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint2")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint3")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/ring/joint0")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/ring/joint1")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/ring/joint2")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint0")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint1")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint2")
        left_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint3")
        N = len(left_hand_group.joint_map.names)
        for i in range(N) :
            left_hand_group.joint_map.ids.append(i)
        left_hand_group.group_id = 0
        left_hand_group.group_name = "left_hand"
        left_hand_group.joint_mask = JointMask()
        left_hand_group.joint_mask.mask = [True]*N
        left_hand_group.control_frame = "r2/left_palm"
        left_hand_group.planning_frame = "/world"

        right_hand_group = PlanGroupConfiguration()
        right_hand_group.joint_map = JointNameMap()
        right_hand_group.joint_map.names = []
        right_hand_group.joint_map.ids = []
        right_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint0")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint1")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint2")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/index/joint3")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/little/joint0")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/little/joint1")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/little/joint2")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint0")  
        right_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint1")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint2")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/middle/joint3")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/ring/joint0")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/ring/joint1")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/ring/joint2")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint0")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint1")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint2")
        right_hand_group.joint_map.names.append("r2/left_arm/hand/thumb/joint3")
        N = len(right_hand_group.joint_map.names)
        for i in range(N) :
            right_hand_group.joint_map.ids.append(i)
        right_hand_group.group_id = 0
        right_hand_group.group_name = "right_hand"
        right_hand_group.joint_mask = JointMask()
        right_hand_group.joint_mask.mask = [True]*N
        right_hand_group.control_frame = "r2/left_palm"
        right_hand_group.planning_frame = "/world"

        response.group_configurations.append(left_arm_group)
        response.group_configurations.append(right_arm_group)

        response.group_configurations.append(left_hand_group)
        response.group_configurations.append(right_hand_group)

        return response

    def handle_plan_command(self, req) :
        rospy.loginfo("PlanCommand()")
        print req
        resp = PlanCommandResponse()
        return resp

    def handle_execute_command(self, req) :
        rospy.loginfo("ExecuteCommand()")
        print req
        resp = ExecuteCommandResponse()
        resp.progress.append(1.0)
        return resp

if __name__=="__main__":

    ads = AtlasDummyService()
    
    rospy.init_node("atlas_dummy")
    rospy.spin()