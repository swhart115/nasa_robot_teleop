#include "RVizInteractiveControlsPanel.hpp"

using namespace rviz_interactive_controls_panel;
using namespace std;

RVizInteractiveControlsPanel::RVizInteractiveControlsPanel(QWidget *parent)
    : rviz::Panel(parent)
    , ui(new Ui::RVizInteractiveControlsPanel)
    , initialized(false)
    , multi_group_widget(NULL)
    , navigation_widget(NULL)
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
    group_widgets.clear();
}

void RVizInteractiveControlsPanel::updateFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &rsp) {
    // NOTE: ignores the return value!
    setupFromConfigResponse(rsp);
}

void RVizInteractiveControlsPanel::setupWidgets() {
    QObject::connect(ui->refresh_button, SIGNAL(clicked()), this, SLOT(getConfigData()));
    QObject::connect(ui->add_group_button, SIGNAL(clicked()), this, SLOT(addGroupRequest()));
    QObject::connect(ui->remove_group_button, SIGNAL(clicked()), this, SLOT(removeGroupRequest()));
    QObject::connect(ui->active_group_list, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(groupDoubleClicked(QListWidgetItem*)));
}

bool RVizInteractiveControlsPanel::setupFromConfigResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp) {
    ROS_INFO("RVizInteractiveControlsPanel::setupFromConfigResponse()");
    ROS_DEBUG("RVizInteractiveControlsPanel: have [%lu] previous groups", previous_groups.size());
    // set up widgets for all the groups
    // TODO: what if a non-active group was removed? do we care?
    ui->all_group_list->clear();
    for (auto& g: resp.group_name) {
        ui->all_group_list->addItem(QString(g.c_str()));
    }
    // set up widget for active groups
    ui->active_group_list->clear();
    std::map<std::string, int>::iterator pgit;
    for (auto& g: resp.active_group_name) {
        // reconcile resp.groups with previous_groups map (for tabs)
        // - groups in map also in resp stay around
        if ((pgit=previous_groups.find(g)) != previous_groups.end()) {
            previous_groups.erase(pgit);
        }
    }
    ROS_DEBUG("RVizInteractiveControlsPanel: got [%lu] removed groups", previous_groups.size());
    // remove/delete groups still in previous_groups
    for (auto pg : previous_groups) {
        const std::string &g = pg.first;
        int idx = ui->GroupTabs->indexOf(group_widgets[g]);
        ui->GroupTabs->removeTab(idx);
        group_widgets.erase(g);
        if (multi_group_widget) {
            multi_group_widget->removeGroup(g);
        }
    }
    ROS_DEBUG("RVizInteractiveControlsPanel: cleared ui lists");

    //std::cout << "RVizInteractiveControlsPanel: response:" << std::endl;
    //std::cout << InteractiveControlsInterfaceUtils::responseStr(resp);
    updateNavigationControls(resp);
    ROS_DEBUG("RVizInteractiveControlsPanel: nav controls updated");
    
    // close old group widgets
    for (auto& gw : group_widgets) {
        auto result = std::find(std::begin(resp.active_group_name), std::end(resp.active_group_name), gw.first);
        //if (
        if(result == std::end(resp.active_group_name)) {
            int index = ui->GroupTabs->indexOf(group_widgets[gw.first]);
            if (index >= 0) {
                ui->GroupTabs->removeTab(index);
                ui->GroupTabs->show();
            }
            if (group_widgets.count(gw.first) > 0) {
                delete group_widgets[gw.first];
            }
        }
    }
    ROS_DEBUG("RVizInteractiveControlsPanel: closed old group widgets");
    
    for(uint idx=0; idx<resp.active_group_name.size(); idx++) {

        std::string group_name = resp.active_group_name[idx];
        ui->active_group_list->addItem(QString(group_name.c_str()));

        addGroupControls(group_name);
        ROS_DEBUG("RVizInteractiveControlsPanel: added group [%s]", group_name.c_str());

        int jdx=0;
        group_widgets[group_name]->group_name = group_name;
        group_widgets[group_name]->joint_names.clear();
        group_widgets[group_name]->joint_mask.clear();
        group_widgets[group_name]->last_sent_joint_mask.clear();

        if(idx < resp.joint_names.size()) {         
            for (auto& j: resp.joint_names[idx].names) {
                group_widgets[group_name]->joint_names.push_back(j);
                group_widgets[group_name]->joint_mask.push_back(resp.joint_mask[idx].mask[jdx]);
                group_widgets[group_name]->last_sent_joint_mask.push_back(resp.joint_mask[idx].mask[jdx]);
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

        if (resp.tolerance_setting.size() > 0) {
            for (auto& tol_info: resp.tolerance_setting[idx].tolerance_info) {
                if(tol_info.mode == "Position Tolerance") {
                    group_widgets[group_name]->position_tolerance = tol_info.types[0];
                } 
                if(tol_info.mode == "Angle Tolerance") {
                    group_widgets[group_name]->orientation_tolerance = tol_info.types[0];
                }
            }
        }

        group_widgets[group_name]->stored_poses.clear();
        if(idx < resp.stored_pose_list.size()) {
            for (auto& stored_pose: resp.stored_pose_list[idx].data) {  
                group_widgets[group_name]->stored_poses.push_back(stored_pose);
            }
        }       
    }

    for(uint idx=0; idx<resp.active_group_name.size(); idx++) {
        std::string group_name = resp.active_group_name[idx];
        group_widgets[group_name]->setupDisplay();
    }
    ROS_DEBUG("RVizInteractiveControlsPanel: group controls updated");

    updateMultiGroupControls(resp);
    ROS_DEBUG("RVizInteractiveControlsPanel: multi-group controls updated");

    initialized = true;

    return true;
}
        

bool RVizInteractiveControlsPanel::addGroupControls(std::string group_name) {

    previous_groups[group_name] = 0; // TODO: should be tab index
    auto search = group_widgets.find(group_name);
    if(search == group_widgets.end()) {
        ROS_DEBUG("RVizInteractiveControlsPanel::addGroupControls(%s)", group_name.c_str());    
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

void RVizInteractiveControlsPanel::updateMultiGroupControls(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp) {
    ROS_INFO("RVizInteractiveControlsPanel::updateMultiGroupControls()");
    if (resp.active_group_name.size() > 1) {
        if (addMultiGroupControls()) {
            // slightly wasteful: just reset and refill, as we're only
            // processing a string/pointer per group
            multi_group_widget->resetGroups();
            for (uint idx=0; idx<resp.active_group_name.size(); ++idx) {
                std::string group_name = resp.active_group_name[idx];
                if (group_widgets.count(group_name) > 0) {
                    multi_group_widget->addGroup(group_name, group_widgets[group_name]);
                }
            }
        }
    } else {
        removeMultiGroupControls();
    }
}

bool RVizInteractiveControlsPanel::addMultiGroupControls() {
    ROS_INFO("RVizInteractiveControlsPanel::addMultiGroupControls()");
    // assume always called after setting up group tabs
    // removing the tab for non-NULL puts it at the end
    const QString label("MG");
    if (multi_group_widget == NULL) {
        ROS_DEBUG("RVizInteractiveControlsPanel: creating MultiGroupControls");
        multi_group_widget = new MultiGroupControlsWidget();
        multi_group_widget->setNodeHandle(nh_);
        multi_group_widget->setServiceClient(&interactive_control_client_);
    } else {
        ROS_DEBUG("RVizInteractiveControlsPanel: remove MultiGroupControls tab");
        int index = ui->GroupTabs->indexOf(multi_group_widget);
        if (index >= 0) {
            ui->GroupTabs->removeTab(index);
        }
    }
    if (multi_group_widget != NULL) {
        ROS_DEBUG("RVizInteractiveControlsPanel: add MultiGroupControls tab");
        ui->GroupTabs->addTab((QWidget *)multi_group_widget, label);
        ui->GroupTabs->show();
    }
    return (multi_group_widget != NULL);
}

bool RVizInteractiveControlsPanel::removeMultiGroupControls() {
    ROS_INFO("RVizInteractiveControlsPanel::removeMultiGroupControls()");
    if (multi_group_widget != NULL) {
        int index = ui->GroupTabs->indexOf(multi_group_widget);
        if (index >= 0) {
            ui->GroupTabs->removeTab(index);
        }
        delete multi_group_widget;
        multi_group_widget = NULL;
        ui->GroupTabs->show();
    }
    return true;
}

void RVizInteractiveControlsPanel::updateNavigationControls(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp) {
    ROS_INFO("RVizInteractiveControlsPanel::updateNavigationControls()");    
    // set up navigation controls widget if applicable
    if (resp.has_navigation_controls) {
        if (addNavigationControls()) {
            navigation_widget->setDataFromResponse(resp);
        }
    } else {
        removeNavigationControls();
    }
}

bool RVizInteractiveControlsPanel::addNavigationControls() {
    ROS_INFO("RVizInteractiveControlsPanel::addNavigationControls()");    
    if (navigation_widget == NULL) {// && initialized) {
        ROS_DEBUG("RVizInteractiveControlsPanel: adding NavigationControls");    
        const QString label("Nav");
        navigation_widget = new NavigationControlsWidget();
        navigation_widget->setNodeHandle(nh_);
        navigation_widget->setServiceClient(&interactive_control_client_);
        ui->GroupTabs->addTab((QWidget *)navigation_widget, label);
        ui->GroupTabs->show();
    }
    return (navigation_widget != NULL);
}

bool RVizInteractiveControlsPanel::removeNavigationControls() {
    ROS_INFO("RVizInteractiveControlsPanel::removeNavigationControls()");
    if (navigation_widget != NULL) {
        ROS_DEBUG("RVizInteractiveControlsPanel: removing NavigationControls");
        int index = ui->GroupTabs->indexOf(navigation_widget);
        if (index >= 0) {
            ui->GroupTabs->removeTab(index);
        }
        delete navigation_widget;
        navigation_widget = NULL;
        ui->GroupTabs->show();
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
        srv.request.group_name.push_back(group_name);
        // TODO: this still isn't right...need a list of groups that were
        //   there *prior* to the service call so we know what (may have)
        //   been deleted. Tab/map removal should be done in the
        //   'setupFromConfigResponse' method.
        //int idx = ui->GroupTabs->indexOf(group_widgets[group_name]);
        //ui->GroupTabs->removeTab(idx);
        //ui->GroupTabs->show();
        //group_widgets.erase(group_name);
        //if (multi_group_widget) {
        //    multi_group_widget->removeGroup(group_name);
        //}
    }
    //std::cout << "RVizInteractiveControlsPanel: remove group request:" << std::endl;
    //std::cout << InteractiveControlsInterfaceUtils::requestStr(srv.request);

    if (interactive_control_client_.call(srv)) {
        setupFromConfigResponse(srv.response);
    } else {
        ROS_ERROR("RVizInteractiveControlsPanel::removeGroupRequest() -- failed to call service");
        return false;
    }

    return true;
}

void RVizInteractiveControlsPanel::groupDoubleClicked(QListWidgetItem* item) {
    selectTab(item->text().toStdString());
}

bool RVizInteractiveControlsPanel::selectTab(const std::string &gn) {
    bool retval = false;
    if (group_widgets.count(gn) > 0) {
        int idx = ui->GroupTabs->indexOf(group_widgets[gn]);
        if (idx >= 0) {
            ui->GroupTabs->setCurrentIndex(idx);
            ui->GroupTabs->show();
            retval = true;
        }
    }
    return retval;
}


#include <pluginlib/class_list_macros.h>
#if ROS_VERSION_MINIMUM(1,9,41)
    PLUGINLIB_EXPORT_CLASS(rviz_interactive_controls_panel::RVizInteractiveControlsPanel, rviz::Panel)
#else
    PLUGINLIB_DECLARE_CLASS(rviz_interactive_controls_panel, RVizInteractiveControlsPanel, rviz_interactive_controls_panel::RVizInteractiveControlsPanel, rviz::Panel)
#endif


