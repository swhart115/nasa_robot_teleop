/**
 * Put copyright notice here
 */
#ifndef INTERACTIVE_CONTROLS_INTERFACE_UTILS_H
#define INTERACTIVE_CONTROLS_INTERFACE_UTILS_H

#include "nasa_robot_teleop/InteractiveControlsInterface.h"
#include "nasa_robot_teleop/JointMask.h"
#include "nasa_robot_teleop/JointNameMap.h"
#include "nasa_robot_teleop/ToleranceInfo.h"
#include "nasa_robot_teleop/ToleranceInfoArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <unordered_set>
#include <vector>

namespace rviz_interactive_controls_panel {

    // typedef for using message in Qt signal/slot; registered in .cpp
    typedef nasa_robot_teleop::InteractiveControlsInterface ICInterface;
    //typedef nasa_robot_teleop::InteractiveControlsInterface::Request InteractiveControlsInterfaceRequest;
    //typedef nasa_robot_teleop::InteractiveControlsInterface::Response InteractiveControlsInterfaceResponse;
    typedef nasa_robot_teleop::InteractiveControlsInterfaceRequest ICInterfaceRequest;
    typedef nasa_robot_teleop::InteractiveControlsInterfaceResponse ICInterfaceResponse;
    
    // TODO: need a service call thread object
    
    class InteractiveControlsInterfaceUtils {
        public:
            static
            void inactiveGroups(const ICInterface::Response &srv,
                                std::unordered_set<std::string> &inactive);
                                //std::vector<std::string> &inactive);
            
            static
            std::string srvStr(const ICInterface &srv, bool suppress=true);
            
            static
            std::string requestStr(const ICInterfaceRequest &srv, bool suppress=true);
            
            static
            std::string responseStr(const ICInterfaceResponse &srv, bool suppress=true);
            
        private:
            static
            std::string actionStr(char type);
            
            static
            std::string vecString2Str(const std::vector<std::string> &vec);
            
            static
            std::string vecPoseStamped2Str(const std::vector<geometry_msgs::PoseStamped> &vec);
            
            static
            std::string vecJointMask2Str(const std::vector<nasa_robot_teleop::JointMask> &vec);
            
            static
            std::string vecToleranceInfo2Str(const std::vector<nasa_robot_teleop::ToleranceInfo> &vec);
            
            //static
            //std::string vecToleranceInfoArray2Str(const std::vector<nasa_robot_teleop::ToleranceInfoArray> &vec);
            
            static
            std::string vecBool2Str(const std::vector<uint8_t> &vec);
            
            static
            std::string vecJointNameMap2Str(const std::vector<nasa_robot_teleop::JointNameMap> &vec);
            
    };
     
}
#endif

