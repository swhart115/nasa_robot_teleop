
import math
import PyKDL as kdl

import geometry_msgs.msg

# from nasa_robot_teleop.moveit_interface import *
# from nasa_robot_teleop.kdl_posemath import *
# from nasa_robot_teleop.pose_update_thread import *
# from nasa_robot_teleop.end_effector_helper import *
from nasa_robot_teleop.urdf_parser_py import *

def normalize_vector(v) :
    m = math.sqrt(math.fsum([x*x for x in v]))
    return [x/m for x in v]

def get_x_rotation_frame(theta) :
    T = kdl.Frame()
    T.M.DoRotX(theta)
    return T

def get_y_rotation_frame(theta) :
    T = kdl.Frame()
    T.M.DoRotY(theta)
    return T

def get_z_rotation_frame(theta) :
    T = kdl.Frame()
    T.M.DoRotZ(theta)
    return T

def get_joint_rotation(axis, joint_val) :
    if axis[0] > 0 :
        T_joint = get_x_rotation_frame(joint_val)
    elif axis[0] < 0 :
        T_joint = get_x_rotation_frame(-joint_val)
    elif axis[1] > 0 :
        T_joint = get_y_rotation_frame(joint_val)
    elif axis[1] < 0 :
        T_joint = get_y_rotation_frame(-joint_val)
    elif axis[2] < 0 :
        T_joint = get_z_rotation_frame(-joint_val)
    else :
        T_joint = get_z_rotation_frame(joint_val)
    return T_joint

def link_has_mesh(link) :
    if link.visual :
        if link.visual.geometry :
            if isinstance(link.visual.geometry, Mesh) :
                if link.visual.geometry.filename :
                    return True
    else :
        return False

def link_has_origin(link) :
    if link.visual :
        if link.visual.origin :
            return True
    else :
        return False

def link_origin_to_pose(link) :
    p = geometry_msgs.msg.Pose()
    p.orientation.w = 1
    if self.link_has_origin(link) :
        if link.visual.origin.xyz :
            p.position.x = link.visual.origin.xyz[0]
            p.position.y = link.visual.origin.xyz[1]
            p.position.z = link.visual.origin.xyz[2]
        if link.visual.origin.rpy :
            q = (kdl.Rotation.RPY(link.visual.origin.rpy[0],link.visual.origin.rpy[1],link.visual.origin.rpy[2])).GetQuaternion()
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
    return p

def joint_origin_to_pose(joint) :
    p = geometry_msgs.msg.Pose()
    p.orientation.w = 1
    if joint.origin :
        if joint.origin.xyz :
            p.position.x = joint.origin.xyz[0]
            p.position.y = joint.origin.xyz[1]
            p.position.z = joint.origin.xyz[2]
        if joint.origin.rpy :
            q = (kdl.Rotation.RPY(joint.origin.rpy[0],joint.origin.rpy[1],joint.origin.rpy[2])).GetQuaternion()
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
    return p

def get_parent_link(link, urdf) :
    for j in urdf.joint_map :
        if urdf.joint_map[j].child == link :
            return urdf.joint_map[j].parent

def get_link_joint(link, urdf) :
    for j in urdf.joint_map :
        if urdf.joint_map[j].child == link :
            return j