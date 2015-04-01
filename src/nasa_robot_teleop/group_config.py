#! /usr/bin/env python

class GroupConfig(object):
    def __init__(self):
    	self.name = ""
    	self.type = "cartesian"
    	self.joint_list = []
    	self.root_frame = ""
    	self.control_frame = ""
    	