#ifndef RVIZ_INTERACTIVE_CONTROLS_PANEL_HPP
#define RVIZ_INTERACTIVE_CONTROLS_PANEL_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>

#include "nasa_robot_teleop/InteractiveControlsInterface.h"

#include "ui_rviz_interactive_controls_panel.h"
#include "InteractiveControlsInterfaceUtils.hpp"
#include "GroupControlsWidget.hpp"
#include "MultiGroupControlsWidget.hpp"
#include "NavigationControlsWidget.hpp"

namespace Ui {
class RVizInteractiveControlsPanel;
}

namespace rviz_interactive_controls_panel {

    class RVizInteractiveControlsPanel : public rviz::Panel
    {
        Q_OBJECT

    public:
        explicit RVizInteractiveControlsPanel(QWidget *parent = 0);
        ~RVizInteractiveControlsPanel();
        
      public Q_SLOTS:  
         bool getConfigData();
         bool popupParamData();
         bool addGroupRequest();
         bool removeGroupRequest();
         void groupDoubleClicked(QListWidgetItem*);

    private:
        
        // pointer to maink UI panel
        Ui::RVizInteractiveControlsPanel *ui;

        // service client to get/set info
        ros::ServiceClient interactive_control_client_;

        // ros node handle
        ros::NodeHandle nh_;

		  // topic base
		  std::string topic_base_;

        // init flag
        bool initialized;
        
        // array of group widgets
        std::map<std::string, GroupControlsWidget *> group_widgets;
        std::map<std::string, int> previous_groups;
        MultiGroupControlsWidget *multi_group_widget;
        NavigationControlsWidget *navigation_widget;

        // setup widget function
        void setupWidgets();
        bool selectTab(const std::string &text);

        // function add a new group controls tab
        bool addGroupControls(std::string group_name);

        // multi-group controls
        void updateMultiGroupControls(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp);
        bool addMultiGroupControls();
        bool removeMultiGroupControls();

        // navigation controls
        void updateNavigationControls(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp);
        bool addNavigationControls();
        bool removeNavigationControls();

        bool setupFromConfigResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp);

    };

}

#endif // RVIZ_INTERACTIVE_CONTROLS_PANEL_HPP
