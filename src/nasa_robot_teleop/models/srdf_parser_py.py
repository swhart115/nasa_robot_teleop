#! /usr/bin/env python

from lxml import etree
import io
import copy
import rospy
import sensor_msgs.msg

from urdf_helper import *

class SRDFModel :

    def __init__(self, robot, urdf=None) :
        self.srdf_file = None
        self.srdf = None
        self.urdf = urdf
        self.robot_name = robot
        self.reset_model()

    def set_urdf(self, urdf) :
        self.urdf = urdf

    def reset_model(self) :

        self.groups = []
        self.base_links = {}
        self.tip_links = {}
        self.group_links = {}
        self.group_joints = {}
        self.group_states = {}
        self.virtual_joints = {}
        self.end_effectors = {}
        self.group_end_effectors = {}
        self.disable_collisions = {}
        self.is_chain = {}
        self.full_group_joints = {}
        # self.full_group_links = {}
        self.joint_mask = {}

    def get_group_links(self, group) :
        return self.group_links[group]

    def get_group_joints(self, group) :
        return self.group_joints[group]

    def parse_from_file(self, filename) :
        try:
            rospy.logdebug(str("SRDFModel::parse_from_file() -- opening SRDF file: " + filename))
            self.srdf_file = open(filename)
            rospy.logdebug("SRDFModel::parse_from_file() -- opened...")
        except IOError :
            rospy.logerr("SRDFModel::parse_from_file() -- unable to load SRDF file...")
            self.srdf_file.close()
            return False
        self.srdf = self.srdf_file.read()
        return self.parse_srdf(self.srdf)

    def parse_srdf(self, srdf) :
        root = etree.fromstring(srdf)
        if (root.tag == "robot") and (root.attrib['name'] != self.robot_name) :
            rospy.logerr("SRDFModel::parse_srdf() -- robot name mismatch")
            rospy.logerr(str("  looking for: " + self.robot_name))
            rospy.logerr(str("  found      : " + root.attrib['name']))
            return False
        else :
            rospy.loginfo(str("SRDFModel::parse_srdf() found robot: " + self.robot_name))

        for elem in root.getchildren() :
            if elem.tag == "group" :
                group_name = elem.attrib["name"]
                rospy.loginfo(str("SRDFModel::parse_srdf() -- found group: " + group_name))
                self.groups.append(group_name)
                self.group_links[group_name] = []
                self.group_joints[group_name] = []
                self.full_group_joints[group_name] = []
                # self.full_group_links[group_name] = []
                self.group_states[group_name] = dict()
                self.base_links[elem.attrib["name"]] = None
                self.tip_links[elem.attrib["name"]] = None

                for group_elem in elem.getchildren() :
                    
                    if group_elem.tag == "chain" :
                        if "base_link" in group_elem.attrib :
                            self.base_links[elem.attrib["name"]] = group_elem.attrib["base_link"]
                        if "tip_link" in group_elem.attrib :
                            self.tip_links[elem.attrib["name"]] = group_elem.attrib["tip_link"]
                    if group_elem.tag == "link" :
                        self.group_links[group_name].append(group_elem.attrib["name"])
                        # self.group_joints[group_name].append(# get parent joint)
                    if group_elem.tag == "joint" :
                        self.group_joints[group_name].append(group_elem.attrib["name"])
                        # self.group_links[group_name].append(# get child link )

            if elem.tag == "group_state" :

                group_name = elem.attrib["group"]
                group_state_name = elem.attrib["name"]
                self.group_states[group_name][group_state_name] = GroupState(group_name, group_state_name)
                for group_elem in elem.getchildren() :
                    if group_elem.tag == "joint" :
                        self.group_states[group_name][group_state_name].add_joint_val(group_elem.attrib["name"], float(group_elem.attrib["value"]))

            if elem.tag == "virtual_joint" :
                joint_name = elem.attrib["name"]
                self.virtual_joints[joint_name] = VirtualJoint()
                self.virtual_joints[joint_name].name = joint_name
                self.virtual_joints[joint_name].type = elem.attrib["type"]
                self.virtual_joints[joint_name].parent_frame = elem.attrib["parent_frame"]
                self.virtual_joints[joint_name].child_link = elem.attrib["child_link"]

            if elem.tag == "end_effector" :
                ee = elem.attrib["name"]
                self.end_effectors[ee] = EndEffector()
                self.end_effectors[ee].name = ee
                self.end_effectors[ee].group = elem.attrib["group"]
                self.end_effectors[ee].parent_link = elem.attrib["parent_link"]
                self.base_links[elem.attrib["group"]] = elem.attrib["parent_link"]
                if "parent_group" in elem.attrib:
                    self.end_effectors[ee].parent_group = elem.attrib["parent_group"]
                else :
                    self.group_end_effectors[ee].parent_group = None

                ee = elem.attrib["group"]
                self.group_end_effectors[ee] = EndEffector()
                self.group_end_effectors[ee].name = elem.attrib["name"]
                self.group_end_effectors[ee].group = elem.attrib["group"]
                self.group_end_effectors[ee].parent_link = elem.attrib["parent_link"]

                if "parent_group" in elem.attrib:
                    self.group_end_effectors[ee].parent_group = elem.attrib["parent_group"]
                else :
                    self.group_end_effectors[ee].parent_group = None

            if elem.tag == "disable_collisions" :
                l1 = elem.attrib["link1"]
                l2 = elem.attrib["link2"]
                r = elem.attrib["reason"]
                self.disable_collisions[(l1,l2)] = r

        if self.urdf :
            self.expand_with_urdf()
            self.validate_groups_with_urdf()

        # self.print_groups()

        return True

    def validate_groups_with_urdf(self) :
        rospy.loginfo("SRDFModel::validate_groups_with_urdf()")
        groups = copy.deepcopy(self.get_groups())
        for g in groups :
            rospy.loginfo("SRDFModel::validate_groups_with_urdf() -- checking group " + g)
            if len(self.group_joints[g]) == 0 :
                rospy.logwarn("SRDFModel::validate_groups_with_urdf() -- removing group (no joints)" + g)
                self.remove_group(g)
            elif len(self.group_links[g]) == 0 :
                rospy.logwarn("SRDFModel::validate_groups_with_urdf() -- removing group (no links)" + g)
                self.remove_group(g)
            else :
                try :
                    for jnt in self.group_joints[g] :
                        if not jnt in self.urdf.joint_map.keys() :
                            rospy.logwarn("SRDFModel::validate_groups_with_urdf() -- removing group (unknown joints) " + g)
                            self.remove_group(g)
                            break
                except :
                    pass
                try :
                    for lnk in self.group_links[g] :
                        if not lnk in self.urdf.link_map.keys() :
                            rospy.logwarn("SRDFModel::validate_groups_with_urdf() -- removing group (unknown links) " + g)
                            self.remove_group(g)
                            break
                except :
                    pass

    def expand_with_urdf(self) :
        rospy.loginfo("SRDFModel::expanding_with_urdf()")

        for g in self.get_groups() :

            if g not in self.base_links:
                self.base_links[g] = None
            if g not in self.tip_links:
                self.tip_links[g] = None
     
            if not self.base_links[g] :
                # print "oops, no base link for group: ", g
                if len(self.group_joints[g]) > 0 :
                    # print "getting first link from joint: ", self.group_joints[g][0]
                    self.base_links[g] = get_joint_parent_link(self.group_joints[g][0], self.urdf)

            if not self.tip_links[g] :
                # print "oops, no tip link for group: ", g
                if len(self.group_joints[g]) > 0 :
                    # print "getting last link from joint: ", self.group_joints[g][len(self.group_joints[g])-1]
                    self.tip_links[g] = get_joint_child_link(self.group_joints[g][len(self.group_joints[g])-1], self.urdf)

            if not g in self.group_joints :
                self.group_joints[g] = []

            if len(self.group_joints[g]) == 0 :
                if is_chain(self.urdf, self.tip_links[g], self.base_links[g]) :
                    self.group_joints[g] = get_chain(self.urdf, self.base_links[g], self.tip_links[g], joints=True, links=False, fixed=False)
                else :
                    for l in self.group_links[g] :
                        jnt = get_link_joint(l, self.urdf)
                        if not self.urdf.joint_map[jnt].joint_type == 'fixed' :
                            self.group_joints[g].append(jnt) 
                            self.full_group_joints[g].append(jnt)           
                
            if g in self.group_links :
                if not g in self.group_links :
                    self.group_links[g] = []

            if len(self.group_links[g]) == 0 :
                if is_chain(self.urdf, self.tip_links[g], self.base_links[g]) :
                    self.group_links[g] = get_chain(self.urdf, self.base_links[g], self.tip_links[g], joints=False, links=True, fixed=False)
                else :
                    # print "just getting links manually for ", g
                    self.group_links[g] = [get_joint_parent_link(j, self.urdf) for j in self.group_joints[g]]
                    # self.full_group_links[g] = self.group_links[g] 
                    # print [get_joint_child_link(j, self.urdf) for j in self.group_joints[g]]

            self.is_chain[g] = is_chain(self.urdf, self.tip_links[g], self.base_links[g])

            if self.is_chain[g] :
                self.full_group_joints[g] = get_chain(self.urdf, self.base_links[g], self.tip_links[g], joints=True, links=False, fixed=False)
                # self.full_group_links[g] = get_chain(self.urdf, self.base_links[g], self.tip_links[g], joints=False, links=True, fixed=False)                   
            else :
                self.full_group_joints[g] = self.group_joints[g]
                
            if "right_robotiq" in g :
                print " joints (full 1): ", self.full_group_joints[g]
                print " joints (1): ", self.group_joints[g]
             
            self.joint_mask[g] = self.compute_joint_mask(g)
            
            if "right_robotiq" in g :
                print " joints (full 2): ", self.full_group_joints[g]
                print " joints (2): ", self.group_joints[g]

            print "===================="
            print "Group: ", g
            print " base: ", self.base_links[g]
            print " tip: ", self.tip_links[g]
            print " joints (spec): ", self.group_joints[g]
            print " joints (full): ", self.full_group_joints[g]
            print " joint mask: ", self.joint_mask[g]
            print " links  (spec): ", self.group_links[g]
            print " is chain: ", 
            print "====================\n"

    def compute_joint_masks(self) :
         for g in self.get_groups() :
            self.joint_mask = self.compute_joint_mask(g)

    def compute_joint_mask(self, g) :
        
        ordered_list = []
        mask = []

        if not g in self.full_group_joints or not g in self.group_joints:
            return None

        if len(self.group_joints[g]) > len(self.full_group_joints[g]) :
            self.full_group_joints[g] = self.group_joints[g]

        if not self.is_chain[g] :
            for j in self.group_joints[g] :
                mask.append(True)
        else :
            for j in self.full_group_joints[g] :
                if j in self.group_joints[g] :
                    mask.append(True)
                    ordered_list.append(j)
                else :
                    mask.append(False)

            self.group_joints[g] = ordered_list
        return mask
                
    def has_tip_link(self, group) :
        return group in self.tip_links and self.tip_links[group] != None

    def get_tip_link(self, group) :
        if group in self.tip_links and self.tip_links[group] != None :
            return self.tip_links[group]
        else :
            rospy.loginfo(str("No Tip Link for Group " + group))
            return ""

    def get_base_link(self, group) :
        if group in self.base_links and self.base_links[group] != None :
            return self.base_links[group]
        else :
            rospy.logerr(str("No base Link for Group " + group))
            return ""

    def remove_group(self, group) :
        try:
            self.groups.remove(group)
            del self.base_links[group]
            del self.tip_links[group]
            del self.group_links[group]
            del self.group_joints[group]
            del self.group_states[group]
            del self.virtual_joints[group]
            del self.end_effectors[group]
            del self.group_end_effectors[group]
            del self.disable_collisions[group]
            del self.is_chain[group]
            del self.full_group_joints[group]
            # del self.full_group_links[group]
            del self.joint_mask[group]
        except :
            pass

    def get_groups(self) :
        return self.groups

    def get_group_state_list(self, group) :
        if group in self.group_states :
            return self.group_states[group].keys()
        else :
            rospy.logerr(str("Group " + group + " does not exist in SRDF"))
            return []

    def get_group_state(self, group, name) :
        if group in self.group_states :
            if name in self.group_states[group] :
                return self.group_states[group][name]
            else :
                rospy.logerr(str("GroupState " + name + " does not exist in SRDF for group: " + group))
        else :
            rospy.logerr(str("Group " + group + " does not exist in SRDF"))

    def get_end_effector_groups(self) :
        g = []
        for e in self.end_effectors.keys() :
            g.append(self.end_effectors[e].group)
        return g

    def get_end_effector_parent_group(self, ee_group) :
        return self.group_end_effectors[ee_group].parent_group

    def get_joint_mask(self, g) :
        if not g in self.joint_mask :
            return None
        return self.joint_mask[g]

    def set_joint_mask(self, g, mask) :
        self.joint_mask[g] = mask

    # def has_end_effector(self, group) :
    #     for ee in self.group_end_effectors.keys() : 
    #         if self.group_end_effectors[ee].parent_group == group :
    #             return True
    #     return False

    def get_end_effector_link(self, group) :
        try :
            for ee in self.end_effectors.keys() : 
                if self.end_effectors[ee].parent_group == group :
                    return self.end_effectors[ee].parent_link
        except :
            return ""

    def print_group_state(self, group, name) :
        if group in self.group_states :
            if name in self.group_states[group] :
                self.group_states[group][name].print_group_state()
            else :
                rospy.logerr(str("GroupState " + name + " does not exist in SRDF for group: " + group))
        else :
            rospy.logerr(str("Group " + group + " does not exist in SRDF"))

    def print_group_states(self, group) :
        for gs in self.get_group_state_list(group) :
            self.print_group_state(group, gs)

    def print_group_states(self) :
        for g in self.groups :
            for gs in self.get_group_state_list(g) :
                self.print_group_state(g, gs)

    def print_virtual_joints(self) :
        for vj in self.virtual_joints.keys() :
            self.virtual_joints[vj].print_virtual_joint()

    def print_end_effectors(self) :
        for ee in self.end_effectors.keys() :
            self.end_effectors[ee].print_end_effector()

    def print_group_list(self) :
        print "Group List: "
        print self.groups

    def print_group(self, group) :
        print "Group: "
        print "\tname: ", group
        print "\tbase_link: ", self.base_links[group]
        print "\ttip_link: ", self.tip_links[group]
        for l in self.group_links[group] :
            print "\tlink: ", l
        for j in self.group_joints[group] :
            print "\tjoint: ", j

    def print_groups(self) :
        print "Groups: "
        for g in self.groups :
            self.print_group(g)

    def print_collisions(self) :
        for c in self.disable_collisions.keys() :
            print "\tDisable Collisions[", c, "]: ", self.disable_collisions[c]

    def print_model(self, print_collisions=False) :
        self.print_group_list()
        self.print_groups()
        self.print_end_effectors()
        self.print_virtual_joints()
        self.print_group_states()
        if print_collisions : self.print_collisions()

class EndEffector :
    def __init__(self) :
        self.name = ""
        self.group = ""
        self.parent_group = ""
        self.parent_link = ""

    def print_end_effector(self) :
        print "EndEffector:"
        print "\tname: ", self.name
        print "\tgroup: ", self.group
        print "\tparent_link: ", self.parent_link
        print "\tparent_group: ", self.parent_group

class VirtualJoint :
    def __init__(self) :
        self.name = ""
        self.type = ""
        self.parent_frame = ""
        self.child_link = ""

    def print_virtual_joint(self) :
        print "VirtualJoint:"
        print "\tname: ", self.name
        print "\ttype: ", self.type
        print "\tparent_frame: ", self.parent_frame
        print "\tchild_link: ", self.child_link

class GroupState :
    def __init__(self, group, name) :
        self.group = group
        self.name = name
        self.joint_state = {}

    def add_joint_val(self, joint, val) :
        self.joint_state[joint] = val

    def to_joint_state_msg(self) :
        js = sensor_msgs.msg.JointState()
        js.name = []
        js.position = []
        for j in self.joint_state.keys() :
            js.name.append(j)
            js.position.append(self.joint_state[j])
        return js

    def print_group_state(self) :
        print "GroupState: "
        print "\tgroup: ", self.group
        print "\tgroup state: ", self.name
        for j in self.joint_state :
            print "\t\t", j, ": ", self.joint_state[j]


