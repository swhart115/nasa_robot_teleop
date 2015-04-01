#include "RVizInteractiveControlsPanel.hpp"

using namespace rviz_interactive_controls_panel;
using namespace std;

RVizInteractiveControlsPanel::RVizInteractiveControlsPanel(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::RVizInteractiveControlsPanel)
{
    ui->setupUi(this);

    // setup service clients
    interactive_control_client_ = nh_.serviceClient<nasa_robot_teleop::InteractiveControlsInterface>("/interactive_control/configure");
    
    // setup QT widgets
    setupWidgets();

    ui->GroupTabs->removeTab(1);
    ui->GroupTabs->show();

    getConfigData();
}

RVizInteractiveControlsPanel::~RVizInteractiveControlsPanel()
{
    delete ui;
}

void RVizInteractiveControlsPanel::setupWidgets() {

    QObject::connect(ui->refresh_button, SIGNAL(clicked()), this, SLOT(getConfigData()));
    QObject::connect(ui->add_group_button, SIGNAL(clicked()), this, SLOT(addGroupRequest()));
    QObject::connect(ui->remove_group_button, SIGNAL(clicked()), this, SLOT(removeGroupRequest()));

}

bool RVizInteractiveControlsPanel::setupFromConfigResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse resp) {

    ui->all_group_list->clear();
    for (auto& g: resp.group_name) {
        ui->all_group_list->addItem(QString(g.c_str()));
    }
    ui->active_group_list->clear();

    for(uint idx=0; idx<resp.active_group_name.size(); idx++) {
        std::string group_name = resp.active_group_name[idx];
        ui->active_group_list->addItem(QString(group_name.c_str()));

        addGroupControls(group_name);
        
        int jdx=0;
        group_widgets[group_name]->group_name = group_name;
        group_widgets[group_name]->joint_names.clear();
        group_widgets[group_name]->joint_mask.clear();

        if(idx < resp.joint_names.size()) {         
            for (auto& j: resp.joint_names[idx].names) {
                group_widgets[group_name]->joint_names.push_back(j);
                group_widgets[group_name]->joint_mask.push_back(resp.joint_mask[idx].mask[jdx]);
                jdx++;  
            }
        }

        if(idx < resp.group_type.size()) {
            group_widgets[group_name]->group_type = resp.group_type[idx];
        }
        if(idx < resp.path_visualization_mode.size()) {
            group_widgets[group_name]->path_visualization_mode = resp.path_visualization_mode[idx];
        }
        if(idx < resp.plan_on_move.size()) {
            group_widgets[group_name]->plan_on_move = resp.plan_on_move[idx];
        }
        if(idx < resp.execute_on_plan.size()) {
            group_widgets[group_name]->execute_on_plan = resp.execute_on_plan[idx];
        }
        if(idx < resp.plan_found.size()) {
            group_widgets[group_name]->plan_found = resp.plan_found[idx];
        }

        for (auto& tol_mode: resp.tolerance) {
            if(tol_mode.mode == "Position Tolerance") {
                group_widgets[group_name]->position_tolerances.clear();
                for (auto& tol_type: tol_mode.types) {
                    group_widgets[group_name]->position_tolerances.push_back(tol_type);
                }
            }
            if(tol_mode.mode == "Angle Tolerance") {
                group_widgets[group_name]->orientation_tolerances.clear();
                for (auto& tol_type: tol_mode.types) {
                    group_widgets[group_name]->orientation_tolerances.push_back(tol_type);
                }
            }
        }

        for (auto& tol_mode: resp.tolerance_setting) {
            if(tol_mode.mode == "Position Tolerance") {
                group_widgets[group_name]->position_tolerance = tol_mode.types[0];
            } else if(tol_mode.mode == "Angle Tolerance") {
                group_widgets[group_name]->orientation_tolerance = tol_mode.types[0];
            }
        }
        
        group_widgets[group_name]->stored_poses.clear();
        if(idx < resp.stored_pose_list.size()) {
            for (auto& stored_pose: resp.stored_pose_list[idx].data) {  
                group_widgets[group_name]->stored_poses.push_back(stored_pose);
            }
        }       
        group_widgets[group_name]->setupDisplay();
    }   

    return true;
}
        


bool RVizInteractiveControlsPanel::addGroupControls(std::string group_name) {

    auto search = group_widgets.find(group_name);
    if(search == group_widgets.end()) {
        ROS_INFO("RVizInteractiveControlsPanel::addGroupControls(%s)", group_name.c_str());    
        const QString label(group_name.c_str());
        group_widgets[group_name] = new GroupControlsWidget();
        group_widgets[group_name]->setNodeHandle(nh_);
        group_widgets[group_name]->setServiceClient(&interactive_control_client_);       
        ui->GroupTabs->addTab((QWidget *)group_widgets[group_name], label);
        ui->GroupTabs->show();
    } else {
        return false;
    }
    return true;
}


bool RVizInteractiveControlsPanel::getConfigData() {

    ROS_INFO("RVizInteractiveControlsPanel::getConfigData() -- querying all group info");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::GET_INFO;

    if (interactive_control_client_.call(srv))
    {
        ROS_INFO("RVizInteractiveControlsPanel::getConfigData() -- success");
        setupFromConfigResponse(srv.response);
    }
    else
    {
        ROS_ERROR("RVizInteractiveControlsPanel::getConfigData() -- failed to call service to get group info");
        return false;
    }

    return true;
}


bool RVizInteractiveControlsPanel::addGroupRequest() {

    ROS_INFO("RVizInteractiveControlsPanel::addGroupRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::ADD_GROUP;

    std::string group_type = ui->group_type_combo_box->currentText().toStdString();
    QList<QListWidgetItem *> items = ui->all_group_list->selectedItems();
    for (auto& g: items) {
        std::string group_name = g->text().toStdString();       
        std::cout << "adding " << group_name << " (type(" << group_type << ") to add list" << std::endl; 
        srv.request.group_name.push_back(group_name);
        srv.request.group_type.push_back(group_type);
    }

    if (interactive_control_client_.call(srv))
    {           
        setupFromConfigResponse(srv.response);
    }
    else
    {
        ROS_ERROR("RVizInteractiveControlsPanel::addGroupRequest() -- failed to call service");
        return false;
    }

    return true;
}

bool RVizInteractiveControlsPanel::removeGroupRequest() {

    ROS_INFO("RVizInteractiveControlsPanel::removeGroupRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::REMOVE_GROUP;

    QList<QListWidgetItem *> items = ui->active_group_list->selectedItems();
    for (auto& g: items) {
        std::string group_name = g->text().toStdString();
        std::cout << "adding " << group_name << " to remove list" << std::endl; 
        srv.request.group_name.push_back(group_name);

        int idx = ui->GroupTabs->indexOf(group_widgets[group_name]);
        ui->GroupTabs->removeTab(idx);
        ui->GroupTabs->show();
        group_widgets.erase(group_name);

    }
        

    
    if (interactive_control_client_.call(srv))
    {
                
        setupFromConfigResponse(srv.response);
    }
    else
    {
        ROS_ERROR("RVizInteractiveControlsPanel::removeGroupRequest() -- failed to call service");
        return false;
    }

    return true;
}




#include <pluginlib/class_list_macros.h>
#if ROS_VERSION_MINIMUM(1,9,41)
    PLUGINLIB_EXPORT_CLASS(rviz_interactive_controls_panel::RVizInteractiveControlsPanel, rviz::Panel)
#else
    PLUGINLIB_DECLARE_CLASS(rviz_interactive_controls_panel, RVizInteractiveControlsPanel, rviz_interactive_controls_panel::RVizInteractiveControlsPanel, rviz::Panel)
#endif


