#! /usr/bin/env python
import rospy
# from math import pi
import yaml

class Tolerance(object) :

    def __init__(self, filename) :
    
        if not filename :
            filename = "/home/swhart/ros_workspace/catkin_workspace/src/nasa_robot_teleop/config/tolerances.yaml"
        f = open(filename)
        self.tolerances = yaml.load(f.read())
        f.close()    
        print yaml.dump(self.tolerances)

        print self.get_tolerance_vals('OrientationTolerance', 'QUARTER_CYLINDER')
        print self.get_tolerance_mode('PositionTolerance', [0.1,0.1,0.1])
        print self.get_tolerance_modes('PositionTolerance')

    def get_tolerance_mode(self, mode, vals) :
        round_digits = 4
        v1 = [round(v,round_digits) for v in vals]   
        if not mode in self.tolerances : 
            rospy.logerr(str("Tolerance::get_tolerance_mode() -- " + mode + " not in Tolerance set: " + self.tolerances.keys()))
            return "FULL"
        for tol_type in self.tolerances[mode] :
            for t in tol_type.keys() :
                v2 = [round(v,round_digits) for v in tol_type[t]]   
                if v1==v2 :
                    return t       
        return "FULL"

    def get_tolerance_vals(self, mode, tol_type) :
        if not mode in self.tolerances : 
            rospy.logerr(str("Tolerance::get_tolerance_vals() -- " + mode + " not in Tolerance set: " + self.tolerances.keys()))
            return [0,0,0]      
        for tol in self.tolerances[mode] :
            if tol_type in tol.keys() :
                return tol[tol_type]
        return [0,0,0]

    def get_tolerance_modes(self, mode) :
        r = []
        if not mode in self.tolerances : 
            rospy.logerr(str("Tolerance::get_tolerance_modes() -- " + mode + " not in Tolerance set: " + self.tolerances.keys()))
        for tol in self.tolerances[mode] :
            for k in tol.keys() :
                r.append(k)
        return r

if __name__=="__main__":
    t = Tolerance(None)