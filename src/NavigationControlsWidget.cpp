#include "NavigationControlsWidget.hpp"

using namespace rviz_interactive_controls_panel;
using namespace std;

NavigationControlsWidget::NavigationControlsWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::NavigationControls)
    , initialized(false)
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
    QObject::connect(ui->direct_move_button, SIGNAL(clicked()), this, SLOT(directMoveRequest()));

    QObject::connect(ui->accommodate_terrain, SIGNAL(stateChanged(int)), this, SLOT(accommodateTerrainClicked(int)));
    QObject::connect(ui->nav_mode_box, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(navModeChanged(const QString&)));

}

void NavigationControlsWidget::setupDisplay() {

    int index;

    ui->waypoint_list->clear();
    for (auto& wp: waypoint_list) {
        ui->waypoint_list->addItem(wp.c_str());
    }
    
    for (auto& nm: navigation_modes) {
        index = ui->nav_mode_box->findText(QString(nm.c_str()));
        if( index == -1 ) {
            ui->nav_mode_box->addItem(QString(nm.c_str()));
        }
    }

    index = ui->nav_mode_box->findText(QString(navigation_mode.c_str()));
    if ( index != -1 ) { // -1 for not found
        ui->nav_mode_box->setCurrentIndex(index);
    }

    if(plan_found) {
        ui->plan_label->setText(QString("PLAN FOUND"));
    } else {
        ui->plan_label->setText(QString("NO PLAN"));
    }

    if(accommodate_terrain) {
        ui->accommodate_terrain->setCheckState(Qt::Checked);
    } else {    
        ui->accommodate_terrain->setCheckState(Qt::Unchecked);
    }


    initialized = true;

}

bool NavigationControlsWidget::setDataFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp) {

    waypoint_list.clear();
    for(uint idx=0; idx<resp.navigation_waypoint_name.size(); idx++) {
        waypoint_list.push_back(resp.navigation_waypoint_name[idx]);
    }

    navigation_modes.clear();
    for(uint idx=0; idx<resp.navigation_modes.size(); idx++) {
        navigation_modes.push_back(resp.navigation_modes[idx]);
    }
    navigation_mode = resp.navigation_mode;

    accommodate_terrain = resp.accommodate_terrain_in_navigation;

    setupDisplay();
    return initialized;

}



void NavigationControlsWidget::accommodateTerrainClicked(int d) {
    
    ROS_INFO("NavigationControlsWidget::accommodateTerrainClicked()");    
    accommodate_terrain = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_ACCOMMODATE_TERRAIN_IN_NAVIGATION;
    srv.request.accommodate_terrain_in_navigation = accommodate_terrain;
    srv.request.navigation_mode = ui->nav_mode_box->currentText().toStdString();
    
    if (service_client_->call(srv))
    {
        ROS_INFO("NavigationControlsWidget::accommodateTerrainClicked() -- success");
        setDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("NavigationControlsWidget::accommodateTerrainClicked() -- failed to call service");
    }

}



bool NavigationControlsWidget::planRequest() {

    ROS_INFO("NavigationControlsWidget::planRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::PLAN_NAVIGATION_PATH;
    srv.request.accommodate_terrain_in_navigation = accommodate_terrain;
    srv.request.navigation_mode = ui->nav_mode_box->currentText().toStdString();
   
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


bool NavigationControlsWidget::directMoveRequest() {

    ROS_INFO("NavigationControlsWidget::directMoveRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::EXECUTE_DIRECT_MOVE;
    srv.request.accommodate_terrain_in_navigation = accommodate_terrain;
    srv.request.navigation_mode = ui->nav_mode_box->currentText().toStdString();
   
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
        ROS_INFO("NavigationControlsWidget::directMoveRequest() -- success");
        return setDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("NavigationControlsWidget::directMoveRequest() -- failed to call service");
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

bool NavigationControlsWidget::navModeChanged(const QString&) {

    ROS_INFO("NavigationControlsWidget::navModeChanged()");    
    
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_NAVIGATION_MODE;
      
    if(ui->nav_mode_box->currentText().toStdString() != navigation_mode) {
        srv.request.navigation_mode = ui->nav_mode_box->currentText().toStdString();
        
        if (service_client_->call(srv))
        {
            ROS_INFO("NavigationControlsWidget::navModeChanged() -- success");
        }
        else
        {
            ROS_ERROR("NavigationControlsWidget::navModeChanged() -- failed to call service");
            return false;
        }
    }
    if (srv.request.navigation_mode == "REACTIVE_WALKER") {
        ui->direct_move_button->setEnabled(true);
        ui->plan_button->setEnabled(false);
        ui->execute_button->setEnabled(false);
    } else if (srv.request.navigation_mode == "WALK_CONTROLLER") {
        ui->direct_move_button->setEnabled(true);
        ui->plan_button->setEnabled(true);
        ui->execute_button->setEnabled(true);
    } else {
        ui->direct_move_button->setEnabled(false);
        ui->plan_button->setEnabled(true);
        ui->execute_button->setEnabled(true);
    }
    return true; //setDataFromResponse(srv.response);
}
