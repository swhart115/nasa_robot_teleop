#! /usr/bin/env python

import yaml

class GroupConfigParser(object):
    
    def __init__(self, filename):
        self.groups = {}
        self.load_yaml(filename)

    def load_yaml(self, filename) :
        try:      
            f = open(filename)
            data = yaml.load(f.read())
            self.groups = data

            # sanity check on misformed yamls
            for t in self.groups.keys() :
                if not self.groups[t] : 
                    self.groups[t] = []

            f.close()
        except :
            print "GroupConfigParser::load_yaml() -- failed to load: " + filename

    def save_yaml(self, filename) :
        with open(filename, 'w') as yaml_file:
            yaml_file.write( yaml.dump(self.groups, default_flow_style=False))

    def get_group_map(self) :
        return self.groups

    def get_groups(self, group_type=None) :
        if group_type==None :
            l = []
            for t in self.groups.keys() :
                if self.groups[t] :
                    l = l+self.groups[t]
            g = list(set(l))
            return g
        else :
            if not group_type in self.groups.keys() :
                return []
            return self.groups[group_type]
 
    def add_group(self, group_type, group) :
        if not group_type in self.groups.keys() :
           self.groups[group_type] = []
        if not group in self.groups[group_type] :
            self.groups[group_type].append(group)
        
    def remove_group(self, group_type, group) :
        if group_type in self.groups.keys() :
           if group in self.groups[group_type] :
                self.groups[group_type].remove(group)
    
    def print_info(self) :
        print self.groups

if __name__=="__main__":
    gcp = GroupConfigParser("/home/swhart/ros/catkin_workspace/src/nasa_robot_teleop/config/atlas.yaml")
    gcp.print_info()

    gcp.add_group("joint", "whatever")
    gcp.remove_group("cartesian", "left_arm")
    gcp.save_yaml("/home/swhart/ros/catkin_workspace/src/nasa_robot_teleop/config/atlas2.yaml")