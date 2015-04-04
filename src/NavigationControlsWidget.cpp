#include "NavigationControlsWidget.hpp"

using namespace rviz_interactive_controls_panel;
using namespace std;

NavigationControlsWidget::NavigationControlsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NavigationControls),
    initialized(false)
{
    ui->setupUi(this);
    setupWidgets();
}

NavigationControlsWidget::~NavigationControlsWidget()
{
    delete ui;
}

void NavigationControlsWidget::setupWidgets() {

    QObject::connect(ui->add_button, SIGNAL(clicked()), this, SLOT(addWaypointRequest()));
    QObject::connect(ui->delete_button, SIGNAL(clicked()), this, SLOT(deleteWaypointRequest()));
    QObject::connect(ui->plan_button, SIGNAL(clicked()), this, SLOT(planRequest()));
    QObject::connect(ui->execute_button, SIGNAL(clicked()), this, SLOT(executeRequest()));

    QObject::connect(ui->left_foot_first, SIGNAL(stateChanged(int)), this, SLOT(leftFootFirstClicked(int)));
    QObject::connect(ui->plan_footsteps, SIGNAL(stateChanged(int)), this, SLOT(planFootstepsClicked(int)));

}

void NavigationControlsWidget::setupDisplay() {

    ui->waypoint_list->clear();
    
    for (auto& wp: waypoint_list) {
        ui->waypoint_list->addItem(wp.c_str());
    }
    
    if(plan_found) {
        ui->plan_label->setText(QString("PLAN FOUND"));
    } else {
        ui->plan_label->setText(QString("NO PLAN"));
    }

    if(left_foot_first) {
        ui->left_foot_first->setCheckState(Qt::Checked);
    } else {    
        ui->left_foot_first->setCheckState(Qt::Unchecked);
    }

    if(plan_footsteps) {
        ui->plan_footsteps->setCheckState(Qt::Checked);
    } else {
        ui->plan_footsteps->setCheckState(Qt::Unchecked);
    }

    initialized = true;

}

bool NavigationControlsWidget::setDataFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse resp) {

    waypoint_list.clear();
    for(uint idx=0; idx<resp.navigation_waypoint_name.size(); idx++) {
        waypoint_list.push_back(resp.navigation_waypoint_name[idx]);
    }

    plan_footsteps = resp.plan_footsteps;
    left_foot_first = resp.left_foot_first;

    setupDisplay();
    return initialized;

}


void NavigationControlsWidget::planFootstepsClicked(int d) {
    
    ROS_INFO("NavigationControlsWidget::planFootstepsClicked()");    
    plan_footsteps = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::PLAN_FOOTSTEPS_IN_PATH;
    srv.request.plan_footsteps = plan_footsteps;

    if (service_client_->call(srv))
    {
        ROS_INFO("NavigationControlsWidget::planFootstepsClicked() -- success");
        setDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("NavigationControlsWidget::planFootstepsClicked() -- failed to call service");
    }

}

void NavigationControlsWidget::leftFootFirstClicked(int d) {
    
    ROS_INFO("NavigationControlsWidget::leftFootFirstClicked()");    
    left_foot_first = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_FIRST_FOOT;
    srv.request.left_foot_first = left_foot_first;

    if (service_client_->call(srv))
    {
        ROS_INFO("NavigationControlsWidget::leftFootFirstClicked() -- success");
        setDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("NavigationControlsWidget::leftFootFirstClicked() -- failed to call service");
    }
    
}



bool NavigationControlsWidget::planRequest() {

    ROS_INFO("NavigationControlsWidget::planRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::PLAN_NAVIGATION_PATH;
    srv.request.left_foot_first = left_foot_first;
    srv.request.plan_footsteps = plan_footsteps;

    QList<QListWidgetItem *> items = ui->waypoint_list->selectedItems();
    if(items.size()==0) {
        int n = ui->waypoint_list->count();
        QListWidgetItem *last_item = ui->waypoint_list->item(n-1);
        srv.request.navigation_waypoint_name.push_back(last_item->text().toStdString());
    } else {
        for (auto& g: items) {
            srv.request.navigation_waypoint_name.push_back(g->text().toStdString());
        }
    }       

    if (service_client_->call(srv))
    {
        ROS_INFO("NavigationControlsWidget::planRequest() -- success");
        return setDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("NavigationControlsWidget::planRequest() -- failed to call service");
        return false;
    }
}


bool NavigationControlsWidget::executeRequest() {

    ROS_INFO("NavigationControlsWidget::executeRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::EXECUTE_NAVIGATION_PATH;
   
    if (service_client_->call(srv))
    {
        ROS_INFO("NavigationControlsWidget::executeRequest() -- success");
        return setDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("NavigationControlsWidget::executeRequest() -- failed to call service");
        return false;
    }

}


bool NavigationControlsWidget::addWaypointRequest() {

    ROS_INFO("NavigationControlsWidget::addWaypointRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::ADD_NAVIGATION_WAYPOINT;
    
    QList<QListWidgetItem *> items = ui->waypoint_list->selectedItems();
    if(items.size()==0) {
        int n = ui->waypoint_list->count();
        QListWidgetItem *last_item = ui->waypoint_list->item(n-1);
        srv.request.navigation_waypoint_name.push_back(last_item->text().toStdString());
    } else {
        for (auto& g: items) {
            srv.request.navigation_waypoint_name.push_back(g->text().toStdString());
        }
    }       

    if (service_client_->call(srv))
    {
        ROS_INFO("NavigationControlsWidget::addWaypointRequest() -- success");
        return setDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("NavigationControlsWidget::addWaypointRequest() -- failed to call service");
        return false;
    }
}

bool NavigationControlsWidget::deleteWaypointRequest() {

    ROS_INFO("NavigationControlsWidget::deleteWaypointRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::DELETE_NAVIGATION_WAYPOINT;
    
    QListWidgetItem *item = ui->waypoint_list->currentItem();
    if(item) {
        srv.request.navigation_waypoint_name.push_back(item->text().toStdString());
    
        if (service_client_->call(srv))
        {
            ROS_INFO("NavigationControlsWidget::deleteWaypointRequest() -- success");
            return setDataFromResponse(srv.response);
        }
        else
        {
            ROS_ERROR("NavigationControlsWidget::deleteWaypointRequest() -- failed to call service");
            return false;
        }
    } else {
        ROS_INFO("NavigationControlsWidget::deleteWaypointRequest() -- no item selected for deletion");
        return true;
    }
}

