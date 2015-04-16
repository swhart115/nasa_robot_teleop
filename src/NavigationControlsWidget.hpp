#ifndef NAVIGATION_CONTROLS_WIDGET_HPP
#define NAVIGATION_CONTROLS_WIDGET_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <ros/package.h>

#include "nasa_robot_teleop/InteractiveControlsInterface.h"


#include "ui_navigation_controls_widget.h"


namespace Ui {
class NavigationControls;
}

namespace rviz_interactive_controls_panel
{

    class NavigationControlsWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit NavigationControlsWidget(QWidget *parent = 0);
         ~NavigationControlsWidget();

        void setNodeHandle(ros::NodeHandle &nh) {
            nh_ = nh;
        }
        
        void setServiceClient(ros::ServiceClient *client_) { 
            service_client_ = client_;
        }

        void setupDisplay();
        bool setDataFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse resp);

    public Q_SLOTS:

        bool planRequest();
        bool executeRequest();
        bool addWaypointRequest();
        bool deleteWaypointRequest();

        void accommodateTerrainClicked(int d);
        bool navModeChanged(const QString&);

    private:

        // the ui
        Ui::NavigationControls *ui;

        // setup widget function
        void setupWidgets();

        // ros node handle
        ros::NodeHandle nh_;

        ros::ServiceClient *service_client_;

        bool initialized;


     public:

        // group storage info
        bool accommodate_terrain;

        std::vector<std::string> waypoint_list;
        std::vector<std::string> navigation_modes;
        std::string navigation_mode;
        
        bool plan_found;
    };

}

#endif // NAVIGATION_CONTROLS_WIDGET_HPP
