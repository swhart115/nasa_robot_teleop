/**
 * Put copyright notice here
 */
#ifndef MULTI_GROUP_CONTROLS_WIDGET_HPP
#define MULTI_GROUP_CONTROLS_WIDGET_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <ros/package.h>

#include "nasa_robot_teleop/InteractiveControlsInterface.h"

#include "ui_multi_group_controls_widget.h"

namespace Ui {
class MultiGroupControls;
}

namespace rviz_interactive_controls_panel {

    class GroupControlsWidget;

    class MultiGroupControlsWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit MultiGroupControlsWidget(QWidget *parent = 0);
         ~MultiGroupControlsWidget();

        void setNodeHandle(ros::NodeHandle &nh) {
            nh_ = nh;
        }
        
        void setServiceClient(ros::ServiceClient *client_) { 
            service_client_ = client_;
        }

        void resetGroups() {
            group_map.clear();
            ui->group_list->clear();
        }

        bool addGroup(const std::string &group_name,
                      GroupControlsWidget* group_widget);

        void removeGroup(const std::string &group_name);

        //void setupDisplay();
        bool setDataFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp);

    public Q_SLOTS:

        bool planRequest();
        bool executeRequest();

        void planOnMoveClicked(int d);
        void executeOnPlanClicked(int d);
        
    private:

        // the ui
        Ui::MultiGroupControls *ui;
        void setupWidgets();

        // ros node handle
        ros::NodeHandle nh_;
        ros::ServiceClient *service_client_;

        std::map<std::string, GroupControlsWidget*> group_map;
        // NOTE: gcvec is vector of bools; for use with ROS message types,
        //   use unsigned char
        void getCheckedGroups(bool andVal, std::vector<std::string> &gnvec,
                              std::vector<unsigned char> &gcvec);
        bool getChecked(const QString &gn);
        QListWidgetItem* getListItem(const QString &gn);

        bool initialized;

     public:
        bool plan_on_move;
        bool execute_on_plan;
        bool plan_found;
    };

}

#endif // MULTI_GROUP_CONTROLS_WIDGET_HPP
