
from math import *
import rospy

import PyKDL as kdl

import geometry_msgs.msg
import visualization_msgs.msg

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
    try :
        if link.visual :
            if link.visual.geometry :
                if isinstance(link.visual.geometry, Mesh) :
                    if link.visual.geometry.filename :
                        return True
        return False
    except :
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
    if link_has_origin(link) :
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

def get_joint_child_link(joint, urdf) :
    if joint in urdf.joint_map :
        return urdf.joint_map[joint].child
    return None
    
def get_mesh_marker_for_link(link_name, urdf) :
    
    try :
        marker = visualization_msgs.msg.Marker()
        p = geometry_msgs.msg.Pose()

        link = urdf.link_map[link_name]

        marker.header.frame_id = link_name
        marker.header.stamp = rospy.get_rostime()
        marker.ns = link_name

        marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE
        marker.mesh_resource = link.visual.geometry.filename
        marker.mesh_use_embedded_materials = True

        s = [1.0, 1.0, 1.0]
        q = [0.0, 0.0, 0.0, 1.0]
        
        if not link.visual.geometry.scale == None :
            s = link.visual.geometry.scale
        
        if not link.visual.origin.rpy == None:
            q = (kdl.Rotation.RPY(link.visual.origin.rpy[0],link.visual.origin.rpy[1],link.visual.origin.rpy[2])).GetQuaternion()
    
        if not link.visual.origin.xyz == None:
            p.position.x = link.visual.origin.xyz[0]
            p.position.y = link.visual.origin.xyz[1]
            p.position.z = link.visual.origin.xyz[2]
        
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        marker.pose = p

        marker.action = visualization_msgs.msg.Marker.ADD
        marker.scale.x = s[0]
        marker.scale.y = s[1]
        marker.scale.z = s[2]
        marker.text = link_name

    except :
        marker = None

    return marker       


def mag(v):
    mag2 = sum(n * n for n in v)
    return sqrt(mag2)

def sign(val):
    if val > 0:
        return 1
    elif val < 0:
        return -1
    else:
        return 0

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        if mag == 0.: return v
        v = tuple(n / mag for n in v)
    return v

def axis_to_q(v, roll_offset=0):
    x, y, z = v
    roll=pi + roll_offset
    rad = sqrt(x*x+y*y+z*z)
    pitch = atan2(-z,sqrt(x*x+y*y)) #acos(sqrt(x*x+y*y)/rad)
    yaw = atan2(y,x)
    return rpy_to_q((roll,pitch,yaw))

def rpy_to_q(v):
    r, p, y = [x/2 for x in v]
    w = cos(r)*cos(p)*cos(y)+sin(r)*sin(p)*sin(y)
    x = sin(r)*cos(p)*cos(y)-cos(r)*sin(p)*sin(y)
    y = cos(r)*sin(p)*cos(y)+sin(r)*cos(p)*sin(y)
    z = cos(r)*cos(p)*sin(y)-sin(r)*sin(p)*cos(y)
    return x, y, z, w

def q_to_axisangle(q):
    w, v = q[0], q[1:]
    theta = acos(w) * 2.0
    return normalize(v), theta

def q_to_rpy(q):
    x,y,z,w = q
    wx,x = normalize((w,x))
    wy,y = normalize((w,y))
    wz,z = normalize((w,z))
    return sign(x)*2*acos(wx), sign(y)*2*acos(wy), sign(z)*2*acos(wz)

