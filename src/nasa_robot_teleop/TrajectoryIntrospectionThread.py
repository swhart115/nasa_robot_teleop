#! /usr/bin/env python

import threading
import rospy

import sensor_msgs.msg
import trajectory_msgs.msg

class TrajectoryIntrospectionThread(threading.Thread):

    def __init__(self, group, robot_name, timeout):
        super(TrajectoryIntrospectionThread, self).__init__()
        
        self.group = group
        self.timeout = timeout
        self.joint_tolerance = 0.05
        self.goal_tolerance = 0.02
        self.robot_name = robot_name
        self.joint_trajectory = None
        self.is_running = False
        self.success = False
        self.current_joint_state = sensor_msgs.msg.JointState()

        print "robot: ", self.robot_name
        rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)
        rospy.sleep(1)

    def set_joint_trajectory(self, joint_trajectory) :
        self.joint_trajectory = joint_trajectory

    def set_thresholds(self, joint_tolerance=0.05, goal_tolerance=0.02) :
        self.joint_tolerance = joint_tolerance
        self.goal_tolerance = goal_tolerance

    def joint_state_callback(self, data):
        self.current_joint_state = data

    def get_current_joint_state(self) :
        return self.current_joint_state

    def check_running(self) :
        return self.is_running

    def check_success(self) :
        return self.success

    def stripout_group_joint_state(self, js, names) :
        # print "names: ", names 
        js_stripped = sensor_msgs.msg.JointState()
        js_stripped.header = js.header
        for name in names :
            idx = js.name.index(name)
            js_stripped.name.append(name)
            js_stripped.position.append(js.position[idx])
        # print "stripped_js: ",
        # print js_stripped
        return js_stripped

    def compare_joint_positions(self, j1, j2, thresh) :

        diff = 0
        for name in j1.name :
            idx1 = j1.name.index(name)
            jnt1 = j1.position[idx1]

            if not name in j2.name :
                rospy.logerror(str("TrajectoryIntrospectionThread::compare_joint_positions() -- can't find " + name + " in second joint state"))
                return False 

            idx2 = j2.name.index(name)
            jnt2 = j2.position[idx1]

            diff += ((jnt1-jnt2)**2)

        if diff < thresh :
            rospy.loginfo(str("joint difference: " + str(diff)))
            return True
        else :
            rospy.loginfo(str("joint difference: " + str(diff)))
            return False

    def run(self) :

        rospy.loginfo(str("TrajectoryIntrospectionThread::run() -- starting for group: " + self.group))       
        self.is_running = True
        self.success = False

        now_time = rospy.Time.now()
        stop_time = now_time + rospy.Duration(self.timeout)
        # listen to /joint_states
        
        while now_time < stop_time :
           
            if isinstance(self.joint_trajectory, trajectory_msgs.msg.JointTrajectory) :
                js = self.get_current_joint_state()
                jnt_current = self.stripout_group_joint_state(js, self.joint_trajectory.joint_names)
                jnt_goal = sensor_msgs.msg.JointState()
                idx = len(self.joint_trajectory.points)-1
                jnt_goal.name = self.joint_trajectory.joint_names
                jnt_goal.position = self.joint_trajectory.points[idx].positions
            else :
                rospy.logerr("TrajectoryIntrospectionThread::determine_execution_success() -- input JointTrajectory invalid type")
                break

            r = self.compare_joint_positions(jnt_current, jnt_goal, thresh=self.joint_tolerance)
            
            if r : break

            now_time = rospy.Time.now()
            rospy.sleep(0.2)

        # compute end-effector position
        # compare last end_effector positon in (joint) trajectory to determine if at goal
        # compare last joint positon in (joint) trajectory to determine if at goal
        # wait for timeout

        rospy.loginfo(str("TrajectoryIntrospectionThread::determine_execution_success() finished with return value: " + str(r)))

        self.success = r
        self.is_running = False

