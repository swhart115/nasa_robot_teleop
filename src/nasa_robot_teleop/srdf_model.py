#! /usr/bin/env python

from lxml import etree
import io

import sensor_msgs.msg

class SRDFModel :

    def __init__(self, robot) :
        self.srdf_file = None
        self.srdf = None
        self.robot_name = robot
        self.reset_model()

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

    def get_group_links(self, group) :
        return self.group_links[group]

    def get_group_joints(self, group) :
        return self.group_joints[group]

    def parse_from_file(self, filename) :
        try:
            print "SRDFModel::parse_from_file() -- opening SRDF file: ", filename
            self.srdf_file = open(filename)
            print "SRDFModel::parse_from_file() -- opened..."
        except IOError :
            print "SRDFModel::parse_from_file() -- unable to load SRDF file..."
            self.srdf_file.close()
            return False
        self.srdf = self.srdf_file.read()
        return self.parse_srdf(self.srdf)

    def parse_srdf(self, srdf) :
        root = etree.fromstring(srdf)
        if (root.tag == "robot") and (root.attrib['name'] != self.robot_name) :
            print "SRDFModel::parse_srdf() -- robot name mismatch"
            print "  looking for: " + self.robot_name
            print "  found      : " + root.attrib['name']
            return False
        else :
            print "SRDFModel::parse_srdf() found robot: " + self.robot_name

        for elem in root.getchildren() :
            if elem.tag == "group" :
                group_name = elem.attrib["name"]
                print "SRDFModel::parse_srdf() -- found group: ", group_name
                self.groups.append(group_name)
                self.group_links[group_name] = []
                self.group_joints[group_name] = []
                self.group_states[group_name] = dict()

                for group_elem in elem.getchildren() :
                    self.base_links[elem.attrib["name"]] = None
                    self.tip_links[elem.attrib["name"]] = None
                    if group_elem.tag == "chain" :
                        if "base_link" in group_elem.attrib :
                            self.base_links[elem.attrib["name"]] = group_elem.attrib["base_link"]
                        if "tip_link" in group_elem.attrib :
                            self.tip_links[elem.attrib["name"]] = group_elem.attrib["tip_link"]
                    if group_elem.tag == "link" :
                        self.group_links[group_name].append(group_elem.attrib["name"])
                    if group_elem.tag == "joint" :
                        self.group_joints[group_name].append(group_elem.attrib["name"])

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

        return True

    def has_tip_link(self, group) :
        return group in self.tip_links and self.tip_links[group] != None

    def get_tip_link(self, group) :
        if group in self.tip_links and self.tip_links[group] != None :
            return self.tip_links[group]
        else :
            print "No Tip Link for Group ", group
            return ""

    def get_groups(self) :
        return self.groups

    def get_group_state_list(self, group) :
        if group in self.group_states :
            return self.group_states[group].keys()
        else :
            print "Group ", group, " does not exist in MoveIt! interface"
            return []

    def get_group_state(self, group, name) :
        if group in self.group_states :
            if name in self.group_states[group] :
                return self.group_states[group][name]
            else :
                print "GroupState ", name, " does not exist in MoveIt! interface for group: ", group
        else :
            print "Group ", group, " does not exist in MoveIt! interface"

    def get_end_effector_groups(self) :
        g = []
        for e in self.end_effectors.keys() :
            g.append(self.end_effectors[e].group)
        return g

    def get_end_effector_parent_group(self, ee_group) :
        return self.group_end_effectors[ee_group].parent_group

    def print_group_state(self, group, name) :
        if group in self.group_states :
            if name in self.group_states[group] :
                self.group_states[group][name].print_group_state()
            else :
                print "GroupState ", name, " does not exist in MoveIt! interface for group: ", group
        else :
            print "Group ", group, " does not exist in MoveIt! interface"

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


