#! /usr/bin/env python


import rospy
import roslib; roslib.load_manifest('nasa_robot_teleop')
import actionlib
from rospkg import RosPack

import tf
import PyKDL as kdl

import copy
import random

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg

from nasa_robot_teleop.msg import *
from nasa_robot_teleop.srv import *
from nasa_robot_teleop.group_config import *

from models.srdf_parser_py import *
from models.urdf_helper import *
import models.urdf_parser_py as urdf

from util.kinematics_util import *
import end_effector_helper as end_effector

class PathPlanner(object):

    def __init__(self, robot_name, config_file):

        self.robot_name = robot_name
        self.config_file = config_file

        self.active_groups = []
        self.group_types = {}
        self.group_controllers = {}

        self.base_frames = {}
        self.control_frames = {}
        self.control_meshes = {}

        # self.control_offset = {}
        self.group_id_offset = {}

        self.trajectory_poses = {}
        self.trajectory_display_markers = {}

        self.end_effector_display = {}
        self.end_effector_map = {}

        self.display_modes = {}
        self.plan_generated = {}
        self.stored_plans = {}
        self.auto_execute = {}
        self.marker_store = {}
        self.command_topics = {}
        self.plan_color = (0.5,0.1,0.75,.5)
        self.path_increment = 1

        self.gripper_action = {}
        self.gripper_client = {}

        self.bridge_topic_map = {}

        self.joint_tolerance = {}
        self.position_tolerances = {}
        self.orientation_tolerances = {}

        self.path_visualization = rospy.Publisher(str('/' + self.robot_name + '/planned_path_visualization'), visualization_msgs.msg.MarkerArray, latch=False, queue_size=10)
        self.joint_state_sub = rospy.Subscriber(str(self.robot_name + "/joint_states"), sensor_msgs.msg.JointState, self.joint_state_callback)
      
        self.tf_listener = tf.TransformListener()
  
        if not self.create_models(config_file) :
            rospy.logerr("PathPlanner::init() -- failed creating RDF models")
            return

    def create_models(self, config_file) :

        rospy.loginfo("PathPlanner::create_models() -- Creating Robot Model from URDF....")
        self.urdf_model = urdf.Robot.from_parameter_server()
        if self.urdf_model == None : return False

        rospy.loginfo("PathPlanner::create_models() -- Creating Robot Model from SRDF....")
        self.srdf_model = SRDFModel(self.robot_name)
        if self.srdf_model == None : return False

        get_all_tips(self.urdf_model)
        self.srdf_model.set_urdf(self.urdf_model)

        for g in self.srdf_model.groups :
            self.joint_names = self.srdf_model.get_joint_names(g)

        try :
            rospy.loginfo(str("PathPlanner::create_models() -- SRDF Filename: " + config_file))
            if self.srdf_model.parse_from_file(config_file) :
                rospy.loginfo("PathPlanner::create_models() -- SRDF created")
            # self.srdf_model.print_groups()
            return True
        except :
            rospy.logerr("PathPlanner()::create_models() -- error parsing SRDF from file")
            return False

    def has_group(self, group_name) :
        return group_name in self.get_group_names()

    def get_group_names(self) :
        return self.active_groups

    def remove_group(self, group_name) :
        try :
            self.active_groups.remove(group_name)
        except :
            pass

    def get_group_type(self, group_name) :
        if self.has_group(group_name) :
            return self.group_types[group_name]
        else :
            return ""

    def get_all_group_joints(self, group_name) :
        if group_name in self.srdf_model.full_group_joints :
            return self.srdf_model.full_group_joints[group_name]
        return []

    def get_urdf_model(self) :
        return self.urdf_model

    def get_srdf_model(self) :
        return self.srdf_model

    def get_end_effector_names(self) :
        return self.get_srdf_model().group_end_effectors.keys() 
        
    def get_control_frame(self, group_name) :
        if not group_name in self.group_types.keys() :
            rospy.logwarn(str("PathPlanner::get_control_frame() -- can't find "+ group_name))
            return ""
        if self.group_types[group_name] == "endeffector" :
            if group_name in self.srdf_model.group_end_effectors :
                return self.srdf_model.group_end_effectors[group_name].parent_link
        elif self.srdf_model.has_tip_link(group_name) :
            return self.srdf_model.get_tip_link(group_name)
        return "world"
    
    def get_stored_group_state(self, group_name, group_state_name) :
        if group_state_name in self.srdf_model.get_group_state_list(group_name) :
            return self.srdf_model.get_group_state(group_name, group_state_name).to_joint_state_msg()
        else :
            return sensor_msgs.msg.JointState()

    def get_stored_state_list(self, group_name) :
        return self.srdf_model.get_group_state_list(group_name)

    def get_control_mesh(self, group_name) :
        return self.control_meshes[group_name]

    def add_planning_group(self, group_name, group_type, joint_tolerance=0.05, position_tolerances=[.02]*3, orientation_tolerances=[.05]*3) :

        rospy.loginfo(str("PathPlanner::add_planning_group() -- " + group_name))

        if not group_name in self.srdf_model.get_groups() :
            rospy.logwarn(str("PathPlanner::add_planning_group() -- skipping " + group_name + "(not valid/active SRDF group)"))
            if group_name in self.active_groups :
                self.active_groups.remove(group_name)
            return False

        if not group_name in self.active_groups  :
            self.active_groups.append(group_name)
    
        self.plan_generated[group_name] = False
        self.stored_plans[group_name] = None
        self.display_modes[group_name] = "all_points"
        self.auto_execute[group_name] = False

        self.group_types[group_name] = group_type
        self.control_frames[group_name] = ""
        self.control_meshes[group_name] = ""
        self.auto_execute[group_name] = False
        self.marker_store[group_name] = visualization_msgs.msg.MarkerArray()
        
        if not self.setup_group(group_name, joint_tolerance, position_tolerances, orientation_tolerances) :
            return False

        try :           

            # generate a random group_id
            self.group_id_offset[group_name] = self.get_random_offset_id()

            # check to see if the group has an associated end effector, and add it if so
            if self.group_types[group_name] == "cartesian" :
                if self.has_end_effector_link(group_name) :
                    ee_link = self.urdf_model.link_map[self.get_end_effector_link(group_name)]
                    self.control_frames[group_name] = self.get_end_effector_link(group_name)
                    for ee in self.srdf_model.end_effectors.keys() :
                        if self.srdf_model.end_effectors[ee].parent_group == group_name :
                            self.end_effector_map[group_name] = ee
                            self.add_planning_group(self.srdf_model.end_effectors[ee].group, "endeffector") 

                else :
                    self.control_frames[group_name] = self.srdf_model.get_tip_link(group_name)
                   
            elif self.group_types[group_name] == "joint" :
                if self.srdf_model.has_tip_link(group_name) :
                    self.control_frames[group_name] = self.srdf_model.get_tip_link(group_name)
                    tip_link = self.urdf_model.link_map[self.control_frames[group_name]]
                    try :
                        self.control_meshes[group_name] = get_child_mesh(self.urdf_model, tip_link)
                    except :
                        rospy.logdebug("PathPlanner::add_planning_group() -- no tip mesh found")
                            
            elif self.group_types[group_name] == "endeffector" :
                self.control_frames[group_name] = self.srdf_model.group_end_effectors[group_name].parent_link
                ee_link = self.urdf_model.link_map[self.control_frames[group_name]]
                try :
                    self.control_meshes[group_name] = ee_link.visual.geometry.filename
                except :
                    self.control_meshes[group_name] = ""
                self.end_effector_display[group_name] = end_effector.EndEffectorHelper(self.robot_name, group_name, self.get_control_frame(group_name), self.tf_listener)
                self.end_effector_display[group_name].populate_data(self.get_group_links(group_name), self.get_urdf_model(), self.get_srdf_model())
             #     if self.has_end_effector_link(group_name) :
            #         ee_link = self.urdf_model.link_map[self.get_end_effector_link(group_name)]
            #     print "ee_link: ", ee_link
            #     try :
            #         self.control_meshes[group_name] = get_child_mesh(self.urdf_model, ee_link)
            #     except :
            #         rospy.logwarn("no mesh found")
                # for ee in self.srdf_model.end_effectors.keys() :
                #     if self.srdf_model.end_effectors[ee].parent_group == group_name :
                #         self.end_effector_map[group_name] = ee
                #         self.add_planning_group(self.srdf_model.end_effectors[ee].group, group_type="endeffector",
                #             joint_tolerance=joint_tolerance, position_tolerances=position_tolerances, orientation_tolerances=orientation_tolerances)

        except :
            rospy.logerr(str("PathPlanner::add_planning_group() -- Robot " + self.robot_name + " has problem setting up group: " + group_name))
            return False          
            
        return True
   
    def get_random_offset_id(self) :
        while True:
            r = int(random.random()*10000000)
            if not r in self.group_id_offset.values() :
                return r


    def get_group_links(self, group) :
        return self.srdf_model.get_group_links(group)

    def get_group_joints(self, group) :
        return self.srdf_model.get_group_joints(group)

    def get_trajectory_display_markers(self, group) :
        if group in self.trajectory_display_markers : return self.trajectory_display_markers[group]
        else : return visualization_msgs.msg.MarkerArray()

    def get_base_frame(self, group) :
        if group in self.base_frames : return self.base_frames[group]
        else : return ""

    def set_base_frame(self, group, base_frame) :
        self.base_frames[group] = base_frame

    def get_group_base_frame(self, group_name) :
        return self.srdf_model.get_base_link(group_name)

    def get_joint_mask(self, group) :
        return self.srdf_model.get_joint_mask(group)

    def set_joint_mask(self, group, mask) :
        self.srdf_model.set_joint_mask(group, mask)

    def set_display_mode(self, group, mode) :
        self.display_modes[group] = mode

    def check_valid_plan(self, plan) :
        r = len(plan) > 0 
        if r :
            rospy.loginfo(str("PathPlanner::check_valid_plan() -- Plan Found, size: " + str(len(plan))))
        else :
            rospy.logwarn(str("PathPlanner::check_valid_plan() -- No Plan Found"))
        return r   

    def lookup_bridge_topic_name(self, controller_name) :
        bridge_topic_name = rospy.get_param(str(controller_name + "/bridge_topic"), "")
        return bridge_topic_name


    #############################
    ##### path pub methods ######
    #############################

    # publish path plan to the visualization topic (a MarkerArray you can view in RViz)
    def publish_path_data(self, jt, group) :
        if jt != None :
            # first clear out the last one cause RViz is terrible
            self.clear_published_path(group)
            # only do it if it is NOT an end-effector
            if self.group_types[group] != "endeffector" :
                path_visualization_marker_array = self.joint_trajectory_to_marker_array(jt, group, self.display_modes[group])
                if len(path_visualization_marker_array.markers) > 0 :
                    self.path_visualization.publish(path_visualization_marker_array)
                

    # publish a dummy MarkerArray so that RViz clears things out
    def clear_published_path(self,group) :
        markers = visualization_msgs.msg.MarkerArray()
        markers.markers = []
        for m in self.marker_store[group].markers :
            marker = copy.deepcopy(m)
            marker.action = visualization_msgs.msg.Marker.DELETE
            markers.markers.append(marker)
        self.path_visualization.publish(markers)


    ###############################
    ##### conversion methods ######
    ###############################

    # convert the JointTractory msg to a MarkerArray msg that can be vizualized in RViz
    def joint_trajectory_to_marker_array(self, joint_trajectory, group, display_mode) :

        print "creating purple viz of joint traj from ", len(joint_trajectory.points), " points. mode = ", display_mode
        markers = visualization_msgs.msg.MarkerArray()
        markers.markers = []
        # joint_start = self.robot.get_current_state().joint_state
        num_points = len(joint_trajectory.points)
        # print "num_points:", num_points
        if num_points == 0 : return markers
        idx = 0

        ee_offset = toPose((0,0,0), (0,0,0,1))

        if display_mode == "all_points" :
            r = joint_trajectory.points[0:num_points:self.path_increment]
        elif display_mode == "last_point" :
            r = [joint_trajectory.points[num_points-1]]

        for point in r :
            waypoint_markers, end_pose, last_link = self.create_marker_array_from_joint_array(group, joint_trajectory.joint_names, point.positions, self.get_group_planning_frame(group), idx, self.plan_color[3])
            idx += self.group_id_offset[group]
            idx += len(waypoint_markers)
            for m in waypoint_markers: markers.markers.append(m)
            try :
                if self.has_end_effector_link(group) and self.group_types[group] == "cartesian":
                    ee_group = self.srdf_model.end_effectors[self.end_effector_map[group]].group
                    ee_root_frame = self.end_effector_display[ee_group].get_root_frame()
                    if last_link != ee_root_frame :
                        self.tf_listener.waitForTransform(last_link, ee_root_frame, rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.tf_listener.lookupTransform(last_link, ee_root_frame, rospy.Time(0))
                        rot = normalize_vector(rot)
                        ee_offset = toPose(trans, rot)

                    offset_pose = toMsg(end_pose*fromMsg(ee_offset))
                    end_effector_markers = self.end_effector_display[ee_group].get_current_position_marker_array(offset=offset_pose, scale=1, color=self.plan_color, root=self.get_group_planning_frame(group), idx=idx)
                    for m in end_effector_markers.markers: markers.markers.append(m)
                    idx += len(end_effector_markers.markers)
            except :
                rospy.logwarn("PathPlanner::joint_trajectory_to_marker_array() -- problem getting end-effector markers")

        self.marker_store[group] = markers
        self.trajectory_display_markers[group] = copy.deepcopy(markers)

        return markers

    def create_marker_array_from_joint_array(self, group, names, joints, root_frame, idx, alpha) :

        markers = []
        T_acc = kdl.Frame()
        T_kin = kdl.Frame()
        now = rospy.get_rostime()

        first_joint = True

        first_joint_name = names[0]
        last_joint_name = names[len(names)-1]
        joint = first_joint_name
     
        parent_link = self.urdf_model.link_map[self.urdf_model.joint_map[first_joint_name].parent]
        last_link = self.urdf_model.link_map[self.get_control_frame(group)]
        # full_names = self.get_joint_chain(parent_link.name, last_link.name)
        full_names = get_chain(self.urdf_model, parent_link.name, last_link.name, joints=True, links=False, fixed=True)

        for joint in full_names :

            # print "Getting maker info for joint ", joint

            marker = visualization_msgs.msg.Marker()
            parent_link = self.urdf_model.link_map[self.urdf_model.joint_map[joint].parent]
            child_link = self.urdf_model.link_map[self.urdf_model.joint_map[joint].child]
            model_joint = self.urdf_model.joint_map[joint]

            try :
                joint_val = joints[names.index(joint)]
                T_joint = get_joint_rotation(model_joint.axis, joint_val)
            except :
                joint_val = 0.0
                T_joint = kdl.Frame()

            if first_joint :
                first_joint = False
                self.tf_listener.waitForTransform(root_frame, parent_link.name, rospy.Time(0), rospy.Duration(5.0))
                (trans, rot) = self.tf_listener.lookupTransform(root_frame, parent_link.name, rospy.Time(0))
                rot = normalize_vector(rot)
                T_acc = fromMsg(toPose(trans,rot))

            T_kin = fromMsg(joint_origin_to_pose(model_joint))
            T_acc = T_acc*T_kin*T_joint

            if link_has_mesh(child_link) or (link_has_shape(child_link) != ""):

                # print "Getting visuals for: ", child_link.name
                T_viz = fromMsg(link_origin_to_pose(child_link))
                T_link = T_acc*T_viz
                marker.pose = toMsg(T_link)
                marker.header.frame_id = root_frame
                marker.header.stamp = now
                marker.ns = self.robot_name
                marker.text = joint
                marker.id = self.group_id_offset[group] + idx

                try :
                    marker.scale.x = child_link.visual.geometry.scale[0]
                    marker.scale.y = child_link.visual.geometry.scale[1]
                    marker.scale.z = child_link.visual.geometry.scale[2]
                except :
                    marker.scale.x = 1
                    marker.scale.y = 1
                    marker.scale.z = 1

                marker.color.r = self.plan_color[0]
                marker.color.g = self.plan_color[1]
                marker.color.b = self.plan_color[2]
                marker.color.a = self.plan_color[3]
                idx += 1

                if link_has_mesh(child_link) :
                    marker.mesh_resource = child_link.visual.geometry.filename
                    marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE

                    # print " has mesh: ", marker.mesh_resource

                else :
                    # print child_link.visual
                    props = get_shape_properties(child_link)

                    # print " has shape: ", link_has_shape(child_link)

                    if props :
                        # print "size props: ", props
                        if link_has_shape(child_link) == "Sphere" :
                            marker.type = visualization_msgs.msg.Marker.SPHERE
                            marker.scale.x *= props
                            marker.scale.y *= props
                            marker.scale.z *= props

                        elif link_has_shape(child_link) == "Box" :
                            marker.type = visualization_msgs.msg.Marker.CUBE
                            marker.scale.x *= props['xyz'][0]
                            marker.scale.y *= props['xyz'][1]
                            marker.scale.z *= props['xyz'][2]

                        elif link_has_shape(child_link) == "Cylinder" :
                            marker.type = visualization_msgs.msg.Marker.CYLINDER
                            marker.scale.x *= props[0]
                            marker.scale.y *= props[0]
                            marker.scale.z *= props[1]


                marker.action = visualization_msgs.msg.Marker.ADD
                marker.mesh_use_embedded_materials = True
                markers.append(marker)

        return markers, T_acc, child_link.name


    def get_subgroup_plan(self, plan, group_name) :

        # extract a trajectory from the plan input that corresponds only to the joints of the group
        group_joints = self.get_group_joints(group_name)
        traj = trajectory_msgs.msg.JointTrajectory()
        jnt_map = {}
        print "group_joints: ", group_joints
        print "plan.joint_names: ", plan.joint_names
        
        for jnt in plan.joint_names :
            traj.joint_names.append(jnt)
            jnt_map[jnt] = plan.joint_names.index(jnt)
        for p in plan.points: 
            new_p = trajectory_msgs.msg.JointTrajectoryPoint()
            for jnt in plan.joint_names :
                new_p.positions.append(p.positions[jnt_map[jnt]])
            new_p.time_from_start = p.time_from_start
            traj.points.append(new_p)       
        # for jnt in group_joints :
        #     if jnt in plan.joint_names :
        #         traj.joint_names.append(jnt)
        #         jnt_map[jnt] = plan.joint_names.index(jnt)
        # for p in plan.points: 
        #     new_p = trajectory_msgs.msg.JointTrajectoryPoint()
        #     for jnt in group_joints :
        #         if jnt in plan.joint_names :
        #             new_p.positions.append(p.positions[jnt_map[jnt]])
        #     new_p.time_from_start = p.time_from_start
        #     traj.points.append(new_p)       

        return traj


    def get_stored_plans_from_result(self, plan, group_names) :
        for group_name in group_names :
            # some planners might reutrn a combined/aggregate plan for all the groups,
            # if so, tease out the plan for just hte group joints and add it to storage
            # alternatively, the planne might return a dictionary of plans
            if type(plan) == dict :
                self.stored_plans[group_name] = plan[group_name]
            elif type(plan) == trajectory_msgs.msg.JointTrajectory : 
                self.stored_plans[group_name] = self.get_subgroup_plan(plan, group_name)
            else :
                rospy.logerr("PathPlanner::get_stored_plans_from_result() -- unknown return type from planner")
                self.stored_plans[group_name] = None
        return self.stored_plans


    def get_joint_traj_from_goal(self, group_name, goal) :
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = self.srdf_model.get_group_joints(group_name)
        p = trajectory_msgs.msg.JointTrajectoryPoint()          
        p.positions = goal.position    
        traj.points.append(p)
        return traj

    #########################################
    ##### planning & execution methods ######
    #########################################

    
    # computes a PathPlanner JointTrajectory to a single joint goal. 
    def create_joint_plan(self, group_names, goals) :

        rospy.loginfo(str("PathPlanner::create_joint_plan() == Robot Name: " + self.robot_name))

        ret = {}
        
        planning_group_names = []
        idx = 0
        # clear flags
        for group_name in group_names :
            rospy.loginfo(str("PathPlanner::create_joint_plan() ===== PathPlanner Group Name: " + group_name))
            rospy.loginfo(str("PathPlanner::create_joint_plan() ===== Generating Joint Plan "))
            self.plan_generated[group_name] = False
            ret[group_name] = False
                   
            if not (self.group_types[group_name] == "endeffector" and (group_name in self.gripper_action.keys())) :
                planning_group_names.append(group_name)
            else :
                ret[group_name] = True
                self.plan_generated[group_name] = True
                self.stored_plans[group_name] = self.get_joint_traj_from_goal(group_name, goals[idx])
            idx += 1

        if len(planning_group_names)==0 :
            return ret

        # call the abstract method
        stored_plan = self.plan_joint_goals(planning_group_names, goals) 
        self.stored_plans = self.get_stored_plans_from_result(stored_plan, planning_group_names)
        
        for group_name in planning_group_names :
            try :
                # check to make sure the plan has a non-0 amount of waypoints
                if self.stored_plans[group_name] :
                    self.plan_generated[group_name] = self.check_valid_plan(self.stored_plans[group_name].points)

                # if the plan was found publish it to be displayed in RViz as a MarkerArray
                if self.plan_generated[group_name] :
                    self.publish_path_data(self.stored_plans[group_name], group_name)

                ret[group_name] = self.plan_generated[group_name]
            except:
                rospy.logwarn("PathPlanner::create_joint_plan() -- no feedback available")

        rospy.loginfo("PathPlanner::create_joint_plan() -- finished")

        return ret


    # This will create a path plan from a list of waypoints.
    # It will also transpose these waypoints to the robot's planning frame. 
    def create_path_plan(self, group_names, goals) :

        rospy.loginfo(str("PathPlanner::create_path_plan() -- Robot Name: " + self.robot_name))

        idx = 0
        waypoints_list = []
        ret = {}

        for group_name in group_names :
            ret[group_name] = False

            rospy.loginfo(str("PathPlanner::create_path_plan() ---- PathPlanner Group Name: " + group_name))
            rospy.loginfo(str("PathPlanner::create_path_plan() ---- Creating Path Plan"))

            waypoints = []
            rospy.loginfo(str("PathPlanner::create_path_plan() -- transforming input waypoint list for " + group_name + " to frame: " + self.get_group_planning_frame(group_name)))
            for p in goals[idx] :
                pt = geometry_msgs.msg.PoseStamped()
                if p.header.frame_id != self.get_group_planning_frame(group_name) :
                    self.tf_listener.waitForTransform(p.header.frame_id, self.get_group_planning_frame(group_name), rospy.Time(0), rospy.Duration(5.0))
                    pt = self.tf_listener.transformPose(self.get_group_planning_frame(group_name), p)
                waypoints.append(copy.deepcopy(pt))               
            waypoints_list.append(waypoints)
            idx += 1

        rospy.loginfo("PathPlanner::create_path_plan() -- planning Cartesian path(s)")
        try :
            stored_plan = self.plan_cartesian_paths(group_names, waypoints_list)       
        except :
            rospy.logerr("PathPlanner::create_path_plan() -- failed planning Cartesian path(s)")
            return ret

        self.stored_plans = self.get_stored_plans_from_result(stored_plan, group_names)
        for group_name in group_names :
            try :
                # check to make sure the plan has a non-0 amount of waypoints
                if self.stored_plans[group_name] :
                    self.plan_generated[group_name] = self.check_valid_plan(self.stored_plans[group_name].points)
                # if the plan was found publish it to be displayed in RViz as a MarkerArray
                if self.plan_generated[group_name] :
                    self.publish_path_data(self.stored_plans[group_name], group_name)
            except:
                rospy.logwarn("PathPlanner::create_path_plan() -- no feedback available")
            ret[group_name] = self.plan_generated[group_name]
        return ret
   

    def plan_joint_and_execute(self, group_names, goals) :
        for group_name in group_names :
            self.auto_execute[group_name] = True
        return self.create_joint_plan(group_names, goals)


    def plan_cartesian_and_execute(self, group_names, goals) :
        for group_name in group_names :
            self.auto_execute[group_name] = True
        return self.create_path_plan(group_names, goals)


    def execute(self, group_names, from_stored=True, wait=False) :
        rospy.loginfo("PathPlanner::execute()")
        planner_groups = []
        ret = {}
        for group_name in group_names :
            rospy.loginfo("PathPlanner::execute() -- " + group_name)
            if self.group_types[group_name] == "endeffector" and self.gripper_client:
                rospy.loginfo("PathPlanner::execute() -- using gripper service")
                ret[group_name] = self.execute_gripper_action(group_name)
            else :
                planner_groups.append(group_name)
        for group_name in planner_groups :
            ret[group_name] = self.execute_plans(planner_groups, from_stored, wait)
        return ret

    ##########################
    ##### print methods ######
    ##########################

    def print_group_info(self, group_name) :
        if self.has_group(group_name) :
            rospy.loginfo(str("============================================================"))
            rospy.loginfo(str("============ Robot Name: " + self.robot_name))
            rospy.loginfo(str("============ Group: " + group_name))
           
            if group_name in self.get_group_names() :
                rospy.loginfo(str("============ Type: " + str(self.group_types[group_name])))
                rospy.loginfo(str("============ PathPlanner Planning Frame: " + str(self.get_group_planning_frame(group_name))))
                rospy.loginfo(str("============ Control Frame: " + str(self.get_control_frame(group_name))))
                rospy.loginfo(str("============ Control Mesh: " + str(self.get_control_mesh(group_name))))
                
                # this is a joke
                print "[INFO] [WallTime: 1426794727.845558] [1565.645000] ============ Cartesian Position Tolerances: ", self.get_goal_position_tolerances(group_name)
                print "[INFO] [WallTime: 1426794727.845558] [1565.645000] ============ Cartesian Orientation Tolerances: ", self.get_goal_orientation_tolerances(group_name)
                print "[INFO] [WallTime: 1426794727.845558] [1565.645000] ============ Cartesian Joint Tolerance: ", self.get_goal_joint_tolerance(group_name)
            
            rospy.loginfo("============================================================")

    def print_basic_info(self) :
        rospy.loginfo(str("============================================================"))
        rospy.loginfo(str("============ Robot Name: " + self.robot_name))
        rospy.loginfo(str("============ Group Names: "))
        for g in self.get_group_names() :
            rospy.loginfo(str("===============: " + g))
        rospy.loginfo(str("============ Planning frame: " + self.get_robot_planning_frame() ))
        rospy.loginfo(str("============================================================"))
        for g in self.get_group_names() :
            self.print_group_info(g)


    ####################################
    ##### gripper service methods ######
    ####################################

    def set_gripper_actions(self, actions) :

        for a in actions :
            try :

                self.gripper_client[a['name']] = actionlib.SimpleActionClient(a['action'], GripperCommandAction)

                rospy.loginfo("PathPlanner::set_gripper_action() -- set_gripper_action(" + a['action'] + ") for group " + a['name'] + " -- looking for server")
                if not self.gripper_client[a['name']].wait_for_server(rospy.Duration(2.0)) :
                    rospy.logerr("PathPlanner::run_gripper_action() -- wait for server timeout")
                    self.clear_gripper_actions()
                    return 
                else :
                    rospy.loginfo("PathPlanner::set_gripper_action() -- set_gripper_action(" + a['name'] + ") -- server found")
                
            except rospy.ROSException as e:
                rospy.logerr("PathPlanner::set_gripper_actions(): " + str(e))
                self.clear_gripper_actions()

    def clear_gripper_actions(self) :
        for a in self.gripper_action.keys() :
            del self.gripper_action[a]

        for a in self.gripper_client.keys() :
            del self.gripper_client[a]   
        
    def execute_gripper_action(self, group_name, from_stored=False, wait=True) :
        r = False
        rospy.loginfo(str("PathPlanner::execute_gripper_action() -- executing plan for group: " + group_name))                
        if self.plan_generated[group_name] and self.stored_plans[group_name] :
            if self.group_types[group_name] == "endeffector" and (group_name in self.gripper_client.keys()):
                r = self.run_gripper_action(group_name, self.stored_plans[group_name])
        else :
            rospy.logerr(str("PathPlanner::execute_gripper_action() -- no plan for group" + group_name + " yet generated."))
            r = False
        rospy.logdebug(str("PathPlanner::execute_gripper_action() -- plan execution: " + str(r)))
        return r

    # for end-effectors that dont take JointTrajectory inputs (e.g., the PR2), this method will let you bypass this
    # to send a service call to any custom node that will interpret the JT as whatever is necessary
    def run_gripper_action(self, group, traj) :
        
        if not group in self.gripper_client.keys() :
            rospy.logerr(str("PathPlanner::run_gripper_action() -- group " + group + " not an available gripper action client"))
            return False

        if not self.gripper_client[group].wait_for_server(rospy.Duration(2.0)) :
            rospy.logerr(str("PathPlanner::run_gripper_action(" + group + ") -- wait for server timeout"))
            return False

        goal = GripperCommandGoal()
        goal.name = group
        goal.pose_name = "end_effector_pose"
        goal.goal_trajectory = traj

        # Fill in the goal here
        self.gripper_client[group].send_goal(goal)

        return True
        # rospy.loginfo("PathPlanner::run_gripper_action() -- polling feedback")
        # fb_msg = rospy.wait_for_message("/run_gripper/server/feedback", GripperCommandActionFeedback, 3.0)

        # while not fb_msg.feedback.planning_complete:
        #     fb_msg = rospy.wait_for_message("/run_gripper/server/feedback", GripperCommandActionFeedback, 3.0)

        # rospy.loginfo("PathPlanner::run_gripper_action() -- PLANNING COMPLETE(?)")
        # if fb_msg.feedback.planning_progress > 0.0 :            
        #     p = self.get_plan()
        #     return p
        # else :
        #     return None


        # if not self.gripper_action :
        #     rospy.logwarn("PathPlanner::run_gripper_action() -- trying to publish, but no gripper service set!")
        #     return False
        # try:
        #     rospy.loginfo("PathPlanner::run_gripper_action() -- calling gripper service")
        #     resp = self.gripper_action(traj, group, "end_effector_pose")
        #     return resp.result
        # except rospy.ServiceException, e:
        #     rospy.logerr("PathPlanner::run_gripper_action() -- gripper service call failed")
        #     return False
 

    ##############################
    ##### tolerance methods ######
    ##############################

    def get_goal_position_tolerances(self, group_name) :
        if not group_name in self.position_tolerances :
            rospy.logwarn(str("PathPlanner::get_goal_position_tolerances(" + group_name +"), group not found!"))
            return [0]*3
        return self.position_tolerances[group_name]
 
    def get_goal_orientation_tolerances(self, group_name) :
        if not group_name in self.orientation_tolerances :
            rospy.logwarn(str("PathPlanner::get_goal_orientation_tolerances(" + group_name +"), group not found!"))
            return [0]*3
        return self.orientation_tolerances[group_name]

    def get_goal_joint_tolerance(self, group_name) :
        if not group_name in self.joint_tolerance :
            rospy.logwarn(str("PathPlanner::get_goal_joint_tolerance(" + group_name +"), group not found!"))
            return [0]*3
        return self.joint_tolerance[group_name]

    def set_goal_position_tolerances(self, group_name, tol) :
        self.position_tolerances[group_name] = tol

    def set_goal_orientation_tolerances(self, group_name, tol) :
        self.orientation_tolerances[group_name] = tol

    def set_goal_joint_tolerance(self, group_name, tol) :
        self.joint_tolerance[group_name] = tol


    #############################
    ##### callback methods ######
    #############################

    def joint_state_callback(self, data):
        self.currentState = data


    ############################
    ##### cleanup methods ######
    ############################

    def tear_down(self) :
        for k in self.end_effector_display.keys() :
            self.end_effector_display[k].stop_offset_update_thread()


    ############################
    ##### virtual methods ######
    ############################

    #### SETUP & HELPER FUNCTIONS
    def setup_group(self, group_name, joint_tolerance, position_tolerances, orientation_tolerances) :
        rospy.logwarn("PathPlanner::setup_group() -- not implemented")
        raise NotImplementedError

    def add_obstacle(self, p, s, n) :
        rospy.logwarn("PathPlanner::add_obstacle() -- collision free path planning not supported")
        raise NotImplementedError

    def get_group_planning_frame(self, group_name) :
        rospy.logerror("PathPlanner::get_group_planning_frame() -- not implemented")
        raise NotImplementedError

    def get_robot_planning_frame(self) :
        rospy.logerror("PathPlanner::get_robot_planning_frame() -- not implemented")
        raise NotImplementedError

    def get_group_joints(self, group_name) :
        rospy.logerror("PathPlanner::get_group_joints() -- not implemented")
        raise NotImplementedError

    def has_end_effector_link(self, group_name) :
        rospy.logerror("PathPlanner::has_end_effector_link() -- not implemented")
        raise NotImplementedError

    def get_end_effector_link(self, group_name) :
        rospy.logerror("PathPlanner::get_end_effector_link() -- not implemented")
        raise NotImplementedError

    def has_joint_map(self, group_name) :
        rospy.logwarn("PathPlanner::has_joint_map() -- not implemented")
        raise NotImplementedError

    def get_joint_map(self, group_name) :
        rospy.logwarn("PathPlanner::get_joint_map() -- not implemented")
        raise NotImplementedError
  
    def get_feet_names(self) :
        rospy.logwarn("PathPlanner::get_feet_names() -- not implemented")
        raise NotImplementedError

    def get_start_foot(self) :
        rospy.logwarn("PathPlanner::get_start_foot() -- not implemented")
        raise NotImplementedError

    def set_start_foot(self, foot) :
        rospy.logwarn("PathPlanner::set_start_foot() -- not implemented")
        raise NotImplementedError

    def clear_goal_targets(self, group_names) :
        rospy.logerror("PathPlanner::clear_goal_target() -- not implemented")
        raise NotImplementedError

    def get_navigation_modes(self) :
        rospy.logerror("PathPlanner::get_navigation_modes() -- not implemented")
        raise NotImplementedError

    def get_navigation_mode(self) :
        rospy.logerror("PathPlanner::get_navigation_mode() -- not implemented")
        raise NotImplementedError

    def set_navigation_mode(self, mode) :
        rospy.logerror("PathPlanner::set_navigation_mode() -- not implemented")
        raise NotImplementedError

    def accommodate_terrain_in_navigation(self) :
        rospy.logerror("PathPlanner::accommodate_terrain_in_navigation() -- not implemented")
        raise NotImplementedError

    def set_accommodate_terrain_in_navigation(self, val) :
        rospy.logerror("PathPlanner::set_accommodate_terrain_in_navigation() -- not implemented")
        raise NotImplementedError

    #### NAVIGATION FUNCTIONS  
    def plan_navigation_path(self, waypoints) :
        rospy.logerror("PathPlanner::plan_navigation_path() -- not implemented")
        raise NotImplementedError


    #### CARTESIAN FUNCTIONS
    def plan_cartesian_goals(self, group_names, goals) :
        rospy.logerror("PathPlanner::plan_cartesian_goals() -- not implemented")
        raise NotImplementedError

    def plan_cartesian_paths(self, group_names, paths) :
        rospy.logerror("PathPlanner::plan_cartesian_paths() -- not implemented")
        raise NotImplementedError


    #### JOINT FUNCTIONS
    def plan_joint_goals(self, group_names, goals) :
        rospy.logerror("PathPlanner::plan_joint_goals() -- not implemented")
        raise NotImplementedError

    def plan_joint_paths(self, group_names) :
        rospy.logerror("PathPlanner::plan_joint_paths() -- not implemented")
        raise NotImplementedError
    

    #### EXCECUTION FUNCTIONS
    def execute_navigation_plan(self, footsteps, lift_heights, feet, goals) :
        rospy.logerror("PathPlanner::execute_navigation_plan() -- not implemented")
        raise NotImplementedError

    def execute_plans(self, group_names, from_stored, wait) :
        rospy.logerror("PathPlanner::execute_plans() -- not implemented")
        raise NotImplementedError

    def direct_move(self, goal) :
        rospy.logerror("PathPlanner::direct_move() -- not implemented")
        raise NotImplementedError
