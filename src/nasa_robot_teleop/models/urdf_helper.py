
from math import *
import rospy

import PyKDL as kdl

import geometry_msgs.msg
import visualization_msgs.msg

from nasa_robot_teleop.models.urdf_parser_py import *

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


def get_chain(urdf, root, tip, joints=True, links=True, fixed=True):
    chain = []
    if links:
        chain.append(tip)
    link = tip
    while link != root:
        if not link in urdf.parent_map : 
            chain = []
            break
        (joint, parent) = urdf.parent_map[link]
        if joints:
            if fixed or urdf.joint_map[joint].joint_type != 'fixed':
                chain.append(joint)
        if links:
            chain.append(parent)
        link = parent
    chain.reverse()
    return chain

def get_root(urdf):
    root = None
    for link in urdf.link_map:
        if link not in urdf.parent_map:
            assert root is None, "Multiple roots detected, invalid URDF."
            root = link
    assert root is not None, "No roots detected, invalid URDF."
    return root

def get_all_tips(urdf) :
    tips = []
    for link in urdf.link_map.keys():
        found_as_parent = False
        for joint in urdf.joint_map:
            if urdf.joint_map[joint].parent == link : 
                found_as_parent = True
                break
        if not found_as_parent : tips.append(link)
    return tips


def get_all_child_links(urdf, root_frame) :

    root_link = urdf.link_map[root_frame]

    j = get_link_joint(root_link, urdf)

    tips = get_all_tips(urdf)
    links = []
    for tip in tips :
        # print "  checking tip: ", tip
        chain = get_chain(urdf, root=root_link.name, tip=tip, joints=False, links=True)
        # print "  found chain: ", chain, "\n"
        for c in chain :
            if c != links: links.append(c)

    # print "all links: "
    r = list(set(links))
    # print "-----\n"
    # print "final set: ", c
    # print "-----"
    return r

