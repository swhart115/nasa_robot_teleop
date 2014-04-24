#! /usr/bin/env python

import sys

import rospy
import roslib; roslib.load_manifest('r2_control')

from rospkg import RosPack

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg

import moveit_commander
import moveit_msgs.msg

import tf
import PyKDL as kdl
from srdf_model import SRDFModel

class MoveItInterface :

    def __init__(self, robot_name, config_package):

        self.robot_name = robot_name
        self.groups = {}
        self.trajectory_publishers = {}

        print "============ Setting up MoveIt! for robot: \'", self.robot_name, "\'"
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.obstacle_markers = visualization_msgs.msg.MarkerArray()
        if not self.create_model(config_package) :
            print "MoveItInterface::init() -- failed creating SRDF model"
            return

        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)
        self.obstacle_publisher = rospy.Publisher(str('/' + self.robot_name + '/obstacle_markers'), visualization_msgs.msg.MarkerArray)
        for g in self.robot.get_group_names() :
            print "MoveItInterface::init() -- adding traj pub for group: ", g
            self.trajectory_publishers[g] = rospy.Publisher(str('/' + self.robot_name + '/' + g + '/move_group/display_planned_path'), moveit_msgs.msg.DisplayTrajectory)
        self.tf_listener = tf.TransformListener()

        self.print_basic_info()

    def create_model(self, config_package) :
        print "============ Creating Robot Model from SRDF...."
        self.srdf_model = SRDFModel(self.robot_name)
        try :
            srdf_filename = str(RosPack().get_path(config_package) + "/config/" + self.robot_name + ".srdf")
            if self.srdf_model.parse_from_file(srdf_filename) :
                self.srdf_model.print_model(False)
                return True
        except :
            print "ERROR creating SRDF model"
            return False

    def add_group(self, group_name) :
        self.groups[group_name] = moveit_commander.MoveGroupCommander(group_name)

    def has_group(self, group_name) :
        return self.robot.has_group(group_name)

    def print_basic_info(self) :
        print "============================================================"
        print "============ Robot Name: %s" % self.robot_name
        print "============ Group Names: ", self.robot.get_group_names()
        print "============ Planning frame: %s" % self.robot.get_planning_frame()
        # print "============ Robot State: %s" % self.robot.get_current_state()
        print "============================================================"

    def get_control_frame(self, group_name) :
        if self.has_group(group_name) :
            return self.groups[group_name].get_end_effector_link()
        else :
            return ""

    def get_planning_frame(self) :
        return self.robot.get_planning_frame()

    def get_stored_group_state(self, group_name, group_state_name) :
        if group_state_name in self.srdf_model.get_group_state_list(group_name) :
            return self.srdf_model.get_group_state(group_name, group_state_name).to_joint_state_msg()
        else :
            return sensor_msgs.msg.JointState()

    def joint_state_callback(self, data):
        self.currentState = data

    def create_joint_plan_to_target(self, group_name, js) :
        print "== Robot Name: %s" % self.robot_name
        print "===== MoveIt! Group Name: ", group_name
        print "===== Generating Joint plan"
        self.groups[group_name].set_joint_value_target(js)
        plan = self.groups[group_name].plan()

        if plan != None :
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self.trajectory_publishers[group_name].publish(display_trajectory)
            return True
        else :
            return False

    def create_plan_to_target(self, group_name, pt) :

        if pt.header.frame_id != self.groups[group_name].get_planning_frame() :
            self.tf_listener.waitForTransform(pt.header.frame_id, self.groups[group_name].get_planning_frame(), rospy.Time(0), rospy.Duration(5.0))
            pt = self.tf_listener.transformPose(self.groups[group_name].get_planning_frame(), pt)

        print "== Robot Name: %s" % self.robot_name
        print "===== MoveIt! Group Name: %s" % group_name
        print "===== Generating plan"
        self.groups[group_name].set_pose_target(pt)
        plan = self.groups[group_name].plan()

        if plan != None :
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self.trajectory_publishers[group_name].publish(display_trajectory)
            return True
        else :
            return False

    def create_random_target(self, group_name) :
        print "== Robot Name: %s" % self.robot_name
        print "===== MoveIt! Group Name: %s" % group_name
        print "===== Generating random plan"
        self.groups[group_name].set_random_target()
        plan = self.groups[group_name].plan()
        if plan != None :
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self.trajectory_publishers[group_name].publish(display_trajectory)
            return True
        else :
            return False

    def execute_plan(self, group_name) :
        r = self.groups[group_name].go(True)
        print "====== Plan Execution: %s" % r
        return r

    def add_collision_object(self, p, s, n) :
        p.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box(n, p, s)

        m = visualization_msgs.msg.Marker()
        m.header.frame_id = p.header.frame_id
        m.type = m.CUBE
        m.action = m.ADD
        m.scale.x = s[0]
        m.scale.y = s[1]
        m.scale.z = s[2]
        m.color.a = 0.8
        m.color.r = 0
        m.color.g = 1
        m.color.b = 0
        m.pose = p.pose
        m.text = n
        m.ns = n
        self.obstacle_markers.markers.append(m)

        print self.obstacle_markers.markers
        self.obstacle_publisher.publish(self.obstacle_markers)

if __name__ == '__main__':

    rospy.init_node('moveit_intefrace_test')

    moveit_commander.roscpp_initialize(sys.argv)

    try:
        moveit_test = MoveItInterface("r2", "r2_moveit_config")
        moveit_test.add_group("right_arm")
        moveit_test.add_group("left_arm")

        q = (kdl.Rotation.RPY(-1.57,0,0)).GetQuaternion()
        pt = geometry_msgs.msg.PoseStamped()
        pt.header.frame_id = "world"
        pt.header.seq = 0
        pt.header.stamp = rospy.Time.now()
        pt.pose.position.x = -0.3
        pt.pose.position.y = -0.5
        pt.pose.position.z = 1.2
        pt.pose.orientation.x = q[0]
        pt.pose.orientation.y = q[1]
        pt.pose.orientation.z = q[2]
        pt.pose.orientation.w = q[3]
        moveit_test.create_plan_to_target(pt, "left_arm")

        q = (kdl.Rotation.RPY(1.57,0,-1.57)).GetQuaternion()
        pt = geometry_msgs.msg.PoseStamped()
        pt.header.frame_id = "world"
        pt.header.seq = 0
        pt.header.stamp = rospy.Time.now()
        pt.pose.position.x = 0.3
        pt.pose.position.y = -0.5
        pt.pose.position.z = 1.2
        pt.pose.orientation.x = q[0]
        pt.pose.orientation.y = q[1]
        pt.pose.orientation.z = q[2]
        pt.pose.orientation.w = q[3]
        moveit_test.create_plan_to_target(pt, "right_arm")

        r1 = moveit_test.execute_plan("left_arm")
        r2 = moveit_test.execute_plan("right_arm")
        if not r1 : rospy.logerr("moveit_test(left_arm) -- couldn't execute plan")
        if not r2 : rospy.logerr("moveit_test(right_arm) -- couldn't execute plan")

        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass




