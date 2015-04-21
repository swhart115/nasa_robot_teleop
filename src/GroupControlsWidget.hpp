#ifndef GROUP_CONTROLS_WIDGET_HPP
#define GROUP_CONTROLS_WIDGET_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <ros/package.h>

#include "nasa_robot_teleop/InteractiveControlsInterface.h"
#include "nasa_robot_teleop/ToleranceInfo.h"

#include "ui_group_controls_widget.h"
#include "ServiceCallWidgetInterface.hpp"


namespace Ui {
class GroupControls;
}

namespace rviz_interactive_controls_panel {

    class GroupControlsWidget
        : public QWidget
        , public ServiceCallWidgetInterface
    {
        Q_OBJECT

    public:
        typedef nasa_robot_teleop::InteractiveControlsInterface ICIface;
        explicit GroupControlsWidget(QWidget *parent = 0);
         ~GroupControlsWidget();
        /** Implement the service call response handling. Note that this
         * is a late addition that simply calls the original response
         * handler method. */
        void updateFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &rsp);

        void setNodeHandle(ros::NodeHandle &nh) {
            nh_ = nh;
        }
        
        void setServiceClient(ros::ServiceClient *client_) { 
            service_client_ = client_;
        }

        void setupDisplay();
        bool setGroupDataFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp);
        void fillPlanRequest(nasa_robot_teleop::InteractiveControlsInterface &srv);

      Q_SIGNALS:
         void sendCall(ICIface);

    public Q_SLOTS:

        bool planRequest();
        bool executeRequest();
        bool toggleJointControlRequest();
        bool storedPoseRequest();

        void planOnMoveClicked(int d);
        void executeOnPlanClicked(int d);
        void jointMaskChanged(QListWidgetItem* item);
        bool positionToleranceChanged(const QString& text);
        bool rotationToleranceChanged(const QString& text);
        
    private:

        // the ui
        Ui::GroupControls *ui;

        // setup widget function
        void setupWidgets();

        // ros node handle
        ros::NodeHandle nh_;

        ros::ServiceClient *service_client_;

        bool initialized;

     public:

        // group storage info
        std::string group_name;
        std::string group_type;
        std::string path_visualization_mode;
        
        std::vector<std::string> joint_names;
        std::vector<bool> joint_mask;
        std::vector<bool> last_sent_joint_mask;
        
        std::vector<std::string> position_tolerances;
        std::vector<std::string> orientation_tolerances;

        std::string position_tolerance;
        std::string orientation_tolerance;

        std::vector<std::string> stored_poses;
        
        bool plan_on_move;
        bool execute_on_plan;

        bool plan_found;
    };

}

#endif // GROUP_CONTROLS_WIDGET_HPP
