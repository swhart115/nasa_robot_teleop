#include "GroupControlsWidget.hpp"
#include "MultiGroupControlsWidget.hpp"
#include "InteractiveControlsInterfaceUtils.hpp"
#include <iostream>

using namespace rviz_interactive_controls_panel;
using namespace std;

MultiGroupControlsWidget::MultiGroupControlsWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MultiGroupControls)
    , initialized(false)
{
    ui->setupUi(this);
    setupWidgets();
}

MultiGroupControlsWidget::~MultiGroupControlsWidget() {
    delete ui;
}

void MultiGroupControlsWidget::setupWidgets() {
    QObject::connect(ui->plan_button, SIGNAL(clicked()),
                     this, SLOT(planRequest()));
    QObject::connect(ui->execute_button, SIGNAL(clicked()),
                     this, SLOT(executeRequest()));

    QObject::connect(ui->plan_on_move, SIGNAL(stateChanged(int)),
                     this, SLOT(planOnMoveClicked(int)));
    QObject::connect(ui->execute_on_plan, SIGNAL(stateChanged(int)),
                     this, SLOT(executeOnPlanClicked(int)));
    
    //QObject::connect(ui->group_list, SIGNAL(itemClicked(QListWidgetItem *)),
    //                 this, SLOT(groupMaskChanged(QListWidgetItem*)));
}

bool MultiGroupControlsWidget::addGroup(const std::string &group_name,
                                        GroupControlsWidget* group_widget) {
    auto search = group_map.find(group_name);
    if (search == group_map.end()) {
        ROS_INFO("MultiGroupControlsWidget::addGroup(%s) ptr=%p",
                group_name.c_str(), group_widget);
        const QString label(group_name.c_str());
        group_map[group_name] = group_widget;
        ui->group_list->addItem(label);
        QListWidgetItem* item = getListItem(label);
        if (item != NULL) {
            item->setCheckState(Qt::Checked);
        } else {
            item->setCheckState(Qt::Unchecked);
        }
    } else {
        return false;
    }
    return true;
}

void MultiGroupControlsWidget::removeGroup(const std::string &group_name) {
    if (group_map.count(group_name) > 0) {
        std::map<std::string, GroupControlsWidget*>::iterator it;
        it = group_map.find(group_name);
        group_map.erase(it);
    }
    for (int gdx=0; gdx<ui->group_list->count(); ++gdx) {
        QListWidgetItem* listItem = ui->group_list->item(gdx);
        if (group_name == listItem->text().toStdString()) {
            ui->group_list->removeItemWidget(listItem);
            break;
        }
    }
}

bool MultiGroupControlsWidget::setDataFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp) {
    bool retval = false;
    std::cout << "MultiGroupControlsWidget: response:" << std::endl;
    std::cout << InteractiveControlsInterfaceUtils::responseStr(resp) << std::endl;
    
    plan_found = (resp.active_group_name.size() > 0 ? true : false);
    for (uint idx=0; idx<resp.active_group_name.size(); ++idx) {
        std::string group_name(resp.active_group_name[idx]);
        ROS_INFO("  group: [%s]", group_name.c_str());
        if (group_map.count(group_name) > 0) {
            ROS_INFO("    setting data (ptr=%p); group plan_found=%s",
                    group_map[group_name],
                     (group_map[group_name]->plan_found ? "true" : "false"));
            // only consider checked groups for plan status display
            if (!getChecked(QString::fromStdString(group_name))) {
                group_map[group_name]->setGroupDataFromResponse(resp);
            } else {
                retval = retval &&
                    group_map[group_name]->setGroupDataFromResponse(resp, QString(" (from Multigroup)"));
                ROS_INFO("    group plan_found=%s",
                     (group_map[group_name]->plan_found ? "true" : "false"));
                plan_found = plan_found && group_map[group_name]->plan_found;
                ROS_INFO("    my plan_found=%s", (plan_found ? "true" : "false"));
            }
        } else {
            ROS_INFO("    missing in group_map!");
        }
    }
    
    if (plan_found) {
        ui->plan_label->setText(QString("PLAN FOUND"));
    } else {
        ui->plan_label->setText(QString("NO PLAN"));
    }
    return retval;
}

bool MultiGroupControlsWidget::planRequest() {
    ROS_INFO("MultiGroupControlsWidget::planRequest()");
    bool retval = false;
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::PLAN_TO_MARKER;
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        if (getChecked(QString::fromStdString(git->first))) {
            git->second->fillPlanRequest(srv);
        }
    }
    std::cout << "MultiGroupControlsWidget: request:" << std::endl;
    std::cout << rviz_interactive_controls_panel::InteractiveControlsInterfaceUtils::requestStr(srv.request) << std::endl;

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::planRequest() -- success");
        // return value is boolean && of all groups' setGroupData
        retval = setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::planRequest() -- failed to call service");
    }
    return retval;
}

bool MultiGroupControlsWidget::executeRequest() {
    ROS_INFO("MultiGroupControlsWidget::executeRequest()");
    bool retval = false;
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::EXECUTE_PLAN;
    std::vector<unsigned char> junk;
    getCheckedGroups(true, srv.request.group_name, junk);

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::executeRequest() -- success");
        retval = setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::executeRequest() -- failed to call service");
    }
    return retval;
}

void MultiGroupControlsWidget::planOnMoveClicked(int d) {
    ROS_INFO("MultiGroupControlsWidget::planOnMoveClicked()");    
    plan_on_move = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_PLAN_ON_MOVE;

    //getCheckedGroups(plan_on_move, srv.request.group_name, srv.request.plan_on_move);
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        srv.request.group_name.push_back(git->first);
        srv.request.plan_on_move.push_back(getChecked(QString::fromStdString(git->first)) && plan_on_move);
    }
    std::cout << "MultiGroupControlsWidget: request:" << std::endl;
    std::cout << InteractiveControlsInterfaceUtils::requestStr(srv.request) << std::endl;

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::planOnMoveClicked() -- success");
        setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::planOnMoveClicked() -- failed to call service");
    }
}

void MultiGroupControlsWidget::executeOnPlanClicked(int d) {
    ROS_INFO("MultiGroupControlsWidget::executeOnPlanClicked()");    
    execute_on_plan = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_EXECUTE_ON_PLAN;
    //getCheckedGroups(execute_on_plan, srv.request.group_name, srv.request.execute_on_plan);
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        srv.request.group_name.push_back(git->first);
        srv.request.plan_on_move.push_back(getChecked(QString::fromStdString(git->first)) && execute_on_plan);
    }
    std::cout << "MultiGroupControlsWidget: request:" << std::endl;
    std::cout << InteractiveControlsInterfaceUtils::requestStr(srv.request) << std::endl;

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::executeOnPlanClicked() -- success");
        setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::executeOnPlanClicked() -- failed to call service");
    }
}

void MultiGroupControlsWidget::getCheckedGroups(bool andVal,
                std::vector<std::string> &gnvec, std::vector<unsigned char> &gcvec) {
    ROS_INFO("MultiGroupControlsWidget::getCheckedGroups()");
    gnvec.clear(); gcvec.clear(); // just to be sure
    
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        gnvec.push_back(git->first);
        gcvec.push_back((unsigned char)(andVal &&
                                        getChecked(QString::fromStdString(git->first))));
    }
}

bool MultiGroupControlsWidget::getChecked(const QString &gn) {
    bool retval = false;
    QListWidgetItem* listItem = getListItem(gn);
    if (listItem != NULL) {
        retval = (listItem->checkState() == Qt::Checked);
    }
    return retval;
}

QListWidgetItem* MultiGroupControlsWidget::getListItem(const QString &gn) {
    QListWidgetItem* retval = NULL;
    // holy schmoley, either me or (Qt) is dumb in this respect...there's
    // really no way to get an item via it's text? fine, loop over all...
    // TODO: set up a map to keep track of the check state
    for (int gdx=0; gdx<ui->group_list->count(); ++gdx) {
        QListWidgetItem* listItem = ui->group_list->item(gdx);
        if (gn == listItem->text()) {
            retval = listItem;
            break;
        }
    }
    return retval;
}

