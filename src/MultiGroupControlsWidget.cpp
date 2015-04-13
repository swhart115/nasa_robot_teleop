#include "GroupControlsWidget.hpp"
#include "MultiGroupControlsWidget.hpp"

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

MultiGroupControlsWidget::~MultiGroupControlsWidget()
{
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
}

bool MultiGroupControlsWidget::addGroup(const std::string &group_name,
                                        GroupControlsWidget* group_widget) {
    auto search = group_map.find(group_name);
    if (search == group_map.end()) {
        ROS_INFO("MultiGroupControlsWidget::addGroup(%s)", group_name.c_str());
        const QString label(group_name.c_str());
        group_map[group_name] = group_widget;
        ui->group_list->addItem(label);
    } else {
        return false;
    }
    return true;
}

bool MultiGroupControlsWidget::planRequest() {
    ROS_INFO("MultiGroupControlsWidget::planRequest()");

    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::PLAN_TO_MARKER;
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        if (getChecked(git->first)) {
            git->second->fillPlanRequest(srv);
        }
    }

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::planRequest() -- success");
        // return value is boolean && of all groups' setGroupData
        bool retval = false;
        for (uint idx=0; idx<srv.response.group_name.size(); ++idx) {
            std::string group_name(srv.response.group_name[idx]);
            if (group_map.count(group_name) > 0) {
                retval = retval &&
                         group_map[group_name]->setGroupDataFromResponse(srv.response);
            }
        }
        return retval;
    } else {
        ROS_ERROR("MultiGroupControlsWidget::planRequest() -- failed to call service");
        return false;
    }
}

bool MultiGroupControlsWidget::executeRequest() {
    ROS_INFO("MultiGroupControlsWidget::executeRequest()");
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::EXECUTE_PLAN;
    std::vector<unsigned char> junk;
    getCheckedGroups(true, srv.request.group_name, junk);

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::executeRequest() -- success");
        bool retval = false;
        for (uint idx=0; idx<srv.response.group_name.size(); ++idx) {
            std::string group_name(srv.response.group_name[idx]);
            if (group_map.count(group_name) > 0) {
                retval = retval &&
                         group_map[group_name]->setGroupDataFromResponse(srv.response);
            }
        }
        return retval;
    } else {
        ROS_ERROR("MultiGroupControlsWidget::executeRequest() -- failed to call service");
        return false;
    }
}

void MultiGroupControlsWidget::planOnMoveClicked(int d) {
    ROS_INFO("MultiGroupControlsWidget::planOnMoveClicked()");    
    plan_on_move = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_PLAN_ON_MOVE;
    getCheckedGroups(plan_on_move, srv.request.group_name, srv.request.plan_on_move);

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::planOnMoveClicked() -- success");
        for (uint idx=0; idx<srv.response.group_name.size(); ++idx) {
            std::string group_name(srv.response.group_name[idx]);
            if (group_map.count(group_name) > 0) {
                group_map[group_name]->setGroupDataFromResponse(srv.response);
            }
        }
    } else {
        ROS_ERROR("MultiGroupControlsWidget::planOnMoveClicked() -- failed to call service");
    }
}

void MultiGroupControlsWidget::executeOnPlanClicked(int d) {
    ROS_INFO("MultiGroupControlsWidget::executeOnPlanClicked()");    
    execute_on_plan = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_EXECUTE_ON_PLAN;
    getCheckedGroups(execute_on_plan, srv.request.group_name, srv.request.execute_on_plan);

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::executeOnPlanClicked() -- success");
        for (uint idx=0; idx<srv.response.group_name.size(); ++idx) {
            std::string group_name(srv.response.group_name[idx]);
            if (group_map.count(group_name) > 0) {
                group_map[group_name]->setGroupDataFromResponse(srv.response);
            }
        }
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
        gcvec.push_back((unsigned char)(andVal && getChecked(git->first)));
    }
}

bool MultiGroupControlsWidget::getChecked(const std::string &gn) {
    bool retval = false;
    // holy schmoley, either me or (Qt) is dumb in this respect...there's
    // really no way to get an item via it's text? fine, loop over all...
    // TODO: set up a map to keep track of the check state
    for (int gdx=0; gdx<ui->group_list->count(); ++gdx) {
        QListWidgetItem* listItem = ui->group_list->item(gdx);
        if (gn == listItem->text().toStdString()) {
            retval = (listItem->checkState() == Qt::Checked);
            break;
        }
    }
    return retval;
}

