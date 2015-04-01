#! /usr/bin/env python

import rospy
import yaml

class Tolerance(object) :

    def __init__(self, filename) :
    
        if not filename :
            rospy.logwarn("No Tolerance File given!!")
            return

        try :
            f = open(filename)
            self.tolerances = yaml.load(f.read())
            f.close()    
            # print yaml.dump(self.tolerances)
        except :
            self.tolerances = None
            print "bad tolerance file!"

        return None
  
    def get_tolerance_mode(self, mode, vals) :
        round_digits = 4
        v1 = [round(vals[0],round_digits), round(vals[1],round_digits), round(vals[2],round_digits)]   
        if not mode in self.tolerances : 
            rospy.logerr(str("Tolerance::get_tolerance_mode() -- " + mode + " not in Tolerance set: " + self.tolerances.keys()))
            if "Position" in mode:
                return "CENTIMETER"
            else :       
                return "EXACT_ANGLE"

        for tol_type in self.tolerances[mode] :
            for t in tol_type.keys() :
                v2 = [round(v,round_digits) for v in tol_type[t]]   
                if v1==v2 :
                    return t  

        if "Position" in mode:
            return "CENTIMETER"
        else :       
            return "EXACT_ANGLE"


    def get_tolerance_vals(self, mode, tol_type) :
        if not mode in self.tolerances : 
            rospy.logerr(str("Tolerance::get_tolerance_vals() -- " + mode + " not in Tolerance set: " + self.tolerances.keys()))
            return [0,0,0]      
        for tol in self.tolerances[mode] :
            if tol_type in tol.keys() :
                return tol[tol_type]
        return [0,0,0]

    def get_tolerance_modes(self) :
        return self.tolerances.keys()

    def get_tolerances(self, mode) :
        r = []
        if not mode in self.tolerances : 
            rospy.logerr(str("Tolerance::get_tolerance_modes() -- " + mode + " not in Tolerance set: " + self.tolerances.keys()))
        for tol in self.tolerances[mode] :
            for k in tol.keys() :
                r.append(k)
        return r

if __name__=="__main__":
    t = Tolerance(None)
