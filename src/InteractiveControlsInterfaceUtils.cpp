/**
 * Put copyright notice here
 */
#include "InteractiveControlsInterfaceUtils.hpp"
#include <algorithm>
#include <iostream>

namespace rviz_interactive_controls_panel {

void InteractiveControlsInterfaceUtils::inactiveGroups(
                                  const ICInterface::Response &srv,
                                  std::unordered_set<std::string> &inactive) {
                                  //std::vector<std::string> &inactive) {
    std::vector<std::string> gns(srv.group_name), act(srv.active_group_name), inact;
    std::sort(gns.begin(), gns.end());
    std::sort(act.begin(), act.end());
    std::vector<std::string>::iterator it;
    //std::unordered_set<std::string>::iterator it;
    it=std::set_difference(gns.begin(), gns.end(), act.begin(), act.end(), inact.begin());
    inact.resize(it-inact.begin());
    inactive.clear();
    inactive.insert(inact.begin(), inact.end());
}

std::string InteractiveControlsInterfaceUtils::srvStr(
                               const ICInterface &srv) {
    std::stringstream oss;
    oss << requestStr(srv.request) << std::endl;
    oss << "---" << std::endl;
    oss << responseStr(srv.response) << std::endl;
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::requestStr(
                               const ICInterface::Request &srv) {
    std::stringstream oss;
    oss << "action_type: " << actionStr(srv.action_type) << std::endl;
    oss << "group_name[]: " << vecString2Str(srv.group_name) << std::endl;
    oss << "group_type[]: " << vecString2Str(srv.group_type) << std::endl;
    oss << "stored_pose_name[]: " << vecString2Str(srv.stored_pose_name) << std::endl;
    oss << "path_visualization_mode[]: " << vecString2Str(srv.path_visualization_mode) << std::endl;
    oss << "goal_pose[]: " << std::endl << vecPoseStamped2Str(srv.goal_pose) << std::endl;
    oss << "joint_mask[]: " << vecJointMask2Str(srv.joint_mask) << std::endl;
    oss << "tolerance[]: " << vecToleranceInfo2Str(srv.tolerance) << std::endl;
    oss << "execute_on_plan[]: " << vecBool2Str(srv.execute_on_plan) << std::endl;
    oss << "plan_on_move[]: " << vecBool2Str(srv.plan_on_move) << std::endl;
    oss << "navigation_waypoint_name[]: " << vecString2Str(srv.navigation_waypoint_name) << std::endl;
    oss << "plan_footsteps: " << (srv.plan_footsteps ? "true" : "false") << std::endl;
    oss << "accommodate_terrain_in_navigation: " << (srv.accommodate_terrain_in_navigation ? "true" : "false") << std::endl;
    oss << "navigation_mode: " << srv.navigation_mode << std::endl;
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::responseStr(
                               const ICInterface::Response &srv) {
    std::stringstream oss;
    oss << "action_type: " << actionStr(srv.action_type) << std::endl;
    oss << "group_name[]: " << vecString2Str(srv.group_name) << std::endl;
    oss << "group_type[]: " << vecString2Str(srv.group_type) << std::endl;
    oss << "active_group_name[]: " << vecString2Str(srv.active_group_name) << std::endl;
    oss << "plan_found[]: " << vecBool2Str(srv.plan_found) << std::endl;
    oss << "joint_mask[]: " << vecJointMask2Str(srv.joint_mask) << std::endl;
    oss << "joint_names[]: " << vecJointNameMap2Str(srv.joint_names) << std::endl;
    oss << "path_visualization_mode[]: " << vecString2Str(srv.path_visualization_mode) << std::endl;
    oss << "tolerance[]: " << vecToleranceInfo2Str(srv.tolerance) << std::endl;
    oss << "tolerance_setting[]: *skipping output*" << std::endl;
    //oss << "tolerance_setting[]: " << vecToleranceInfoArray2Str(srv.response.tolerance_setting) << std::endl;
    oss << "execute_on_plan[]: " << vecBool2Str(srv.execute_on_plan) << std::endl;
    oss << "plan_on_move[]: " << vecBool2Str(srv.plan_on_move) << std::endl;
    oss << "stored_pose_list[]: *skipping output*" << std::endl;
    //oss << "stored_pose_list[]: " << vecStringArray2Str(srv.response.stored_pose_list) << std::endl;
    oss << "has_navigation_controls: " << (srv.has_navigation_controls ? "true" : "false") << std::endl;
    oss << "plan_footsteps: " << (srv.plan_footsteps ? "true" : "false") << std::endl;
    oss << "navigation_waypoint_name[]: " << vecString2Str(srv.navigation_waypoint_name) << std::endl;
    oss << "navigation_mode: " << srv.navigation_mode << std::endl;
    oss << "accommodate_terrain_in_navigation: " << (srv.accommodate_terrain_in_navigation ? "true" : "false") << std::endl;
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::actionStr(char type) {
    switch (type) {
    case ICInterface::Request::GET_INFO:
        return "GET_INFO";
    case ICInterface::Request::ADD_GROUP:
        return "ADD_GROUP";
    case ICInterface::Request::REMOVE_GROUP:
        return "REMOVE_GROUP";
    case ICInterface::Request::SET_MARKER_POSE:
        return "SET_MARKER_POSE";
    case ICInterface::Request::SET_SHOW_PATH:
        return "SET_SHOW_PATH";
    case ICInterface::Request::SET_PLAN_ON_MOVE:
        return "SET_PLAN_ON_MOVE";
    case ICInterface::Request::SET_EXECUTE_ON_PLAN:
        return "SET_EXECUTE_ON_PLAN";
    case ICInterface::Request::SET_TOLERANCES:
        return "SET_TOLERANCES";
    case ICInterface::Request::SET_JOINT_MAP:
        return "SET_JOINT_MAP";
    case ICInterface::Request::EXECUTE_STORED_POSE:
        return "EXECUTE_STORED_POSE";
    case ICInterface::Request::TOGGLE_POSTURE_CONTROLS:
        return "TOGGLE_POSTURE_CONTROLS";
    case ICInterface::Request::SYNC_MARKERS_TO_ROBOT:
        return "SYNC_MARKERS_TO_ROBOT";
    case ICInterface::Request::PLAN_TO_MARKER:
        return "PLAN_TO_MARKER";
    case ICInterface::Request::EXECUTE_PLAN:
        return "EXECUTE_PLAN";
    case ICInterface::Request::ADD_NAVIGATION_WAYPOINT:
        return "ADD_NAVIGATION_WAYPOINT";
    case ICInterface::Request::DELETE_NAVIGATION_WAYPOINT:
        return "DELETE_NAVIGATION_WAYPOINT";
    case ICInterface::Request::PLAN_NAVIGATION_PATH:
        return "PLAN_NAVIGATION_PATH";
    case ICInterface::Request::EXECUTE_NAVIGATION_PATH:
        return "EXECUTE_NAVIGATION_PATH";
    case ICInterface::Request::PLAN_FOOTSTEPS_IN_PATH:
        return "PLAN_FOOTSTEPS_IN_PATH";
    case ICInterface::Request::SET_FIRST_FOOT:
        return "SET_FIRST_FOOT";
    default:
        return "unknown";
    }
}

std::string InteractiveControlsInterfaceUtils::vecString2Str(
                                   const std::vector<std::string> &vec) {
    std::stringstream oss;
    oss << "[ ";
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << vec[i] << " ";
    }
    oss << "]";
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecPoseStamped2Str(
                        const std::vector<geometry_msgs::PoseStamped> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "PoseStamped[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecJointMask2Str(
                        const std::vector<nasa_robot_teleop::JointMask> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "JointMask[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecJointNameMap2Str(
                    const std::vector<nasa_robot_teleop::JointNameMap> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "JointNameMap[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecToleranceInfo2Str(
                   const std::vector<nasa_robot_teleop::ToleranceInfo> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "ToleranceInfo[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

/*
std::string InteractiveControlsInterfaceUtils::vecToleranceInfoArray2Str(
             const std::vector<nasa_robot_teleop::ToleranceInfoArray> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "ToleranceInfoArray[" << i << "]:" << std::endl;
        oss << vecToleranceInfo2Str(vec[i]);
    }
    return oss.str();
}
*/

std::string InteractiveControlsInterfaceUtils::vecBool2Str(
                        const std::vector<uint8_t> &vec) {
    std::stringstream oss;
    oss << "[ ";
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << (vec[i] ? "true" : "false") << " ";
    }
    oss << "]";
    return oss.str();
}

}
