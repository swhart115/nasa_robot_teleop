#include "GroupControlsWidget.hpp"

using namespace rviz_interactive_controls_panel;
using namespace std;

GroupControlsWidget::GroupControlsWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::GroupControls)
    , initialized(false)
{
    ui->setupUi(this);
    setupWidgets();
}

GroupControlsWidget::~GroupControlsWidget()
{
    delete ui;
}

void GroupControlsWidget::setupWidgets() {

    QObject::connect(ui->plan_button, SIGNAL(clicked()), this, SLOT(planRequest()));
    QObject::connect(ui->execute_button, SIGNAL(clicked()), this, SLOT(executeRequest()));
    QObject::connect(ui->toggle_joint_control_button, SIGNAL(clicked()), this, SLOT(toggleJointControlRequest()));
    QObject::connect(ui->go_to_stored_pose_button, SIGNAL(clicked()), this, SLOT(storedPoseRequest()));
    QObject::connect(ui->set_tool_offset_button, SIGNAL(clicked()), this, SLOT(setToolOffsetClicked()));
    QObject::connect(ui->clear_tool_offset_button, SIGNAL(clicked()), this, SLOT(clearToolOffsetClicked()));

    QObject::connect(ui->plan_on_move, SIGNAL(stateChanged(int)), this, SLOT(planOnMoveClicked(int)));
    QObject::connect(ui->execute_on_plan, SIGNAL(stateChanged(int)), this, SLOT(executeOnPlanClicked(int)));

    QObject::connect(ui->pos_tol, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(positionToleranceChanged(const QString&)));
    QObject::connect(ui->rot_tol, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(rotationToleranceChanged(const QString&)));
    
    QObject::connect(ui->joint_list, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(jointMaskChanged(QListWidgetItem*)));

}

void GroupControlsWidget::setupDisplay(QString from) {

    ui->joint_list->clear();
    
    ui->type_label->setText(QString(group_type.c_str()));

    int index;
    int jdx = 0;

    for (auto& j: joint_names) {
        ui->joint_list->addItem(j.c_str());
        QListWidgetItem *item = ui->joint_list->item(jdx);
        if(joint_mask[jdx]) {
            item->setCheckState(Qt::Checked);
        } else {
            item->setCheckState(Qt::Unchecked);
        }
        jdx++;
    }
    
    if(plan_found) {
        ui->plan_label->setText(QString("PLAN FOUND")+ from);
    } else {
        ui->plan_label->setText(QString("NO PLAN")+ from);
    }

    if(group_type=="cartesian") {
        if(plan_on_move) {
            ui->plan_on_move->setCheckState(Qt::Checked);
        } else {
            ui->plan_on_move->setCheckState(Qt::Unchecked);
        }

        for (auto& pt: position_tolerances) {
            index = ui->pos_tol->findText(QString(pt.c_str()));
            if( index == -1 ) {
                ui->pos_tol->addItem(QString(pt.c_str()));
            }
        }
            
        for (auto& rt: orientation_tolerances) {
            index = ui->rot_tol->findText(QString(rt.c_str()));
            if( index == -1 ) {
                ui->rot_tol->addItem(QString(rt.c_str()));
            }
        }

        index = ui->pos_tol->findText(QString(position_tolerance.c_str()));
        if ( index != -1 ) { // -1 for not found
            ui->pos_tol->setCurrentIndex(index);
        }
        index = ui->rot_tol->findText(QString(orientation_tolerance.c_str()));
        if ( index != -1 ) { // -1 for not found
            ui->rot_tol->setCurrentIndex(index);
        }

    } else {
        ui->plan_on_move->setEnabled(false);
        ui->pos_tol->setEnabled(false);
        ui->rot_tol->setEnabled(false);
        ui->viz_type->setEnabled(false);
    }


    ui->stored_pose_list->clear();
    for (auto& sp: stored_poses) {
        ui->stored_pose_list->addItem(QString(sp.c_str()));
    }

    if(execute_on_plan) {
        ui->execute_on_plan->setCheckState(Qt::Checked);
    } else {
        ui->execute_on_plan->setCheckState(Qt::Unchecked);
    }

    initialized = true;

}

bool GroupControlsWidget::setGroupDataFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &resp, QString from) {

    ROS_INFO("GroupControlsWidget: [%s] got response", group_name.c_str());
    for (uint idx=0; idx<resp.active_group_name.size(); idx++) {
        if(group_name == resp.active_group_name[idx]) {
            ROS_INFO("GroupControlsWidget: [%s] set data", group_name.c_str());
            int jdx=0;
            joint_names.clear();
            joint_mask.clear();

            if(idx < resp.joint_names.size()) {         
                for (auto& j: resp.joint_names[idx].names) {
                    joint_names.push_back(j);
                    joint_mask.push_back(resp.joint_mask[idx].mask[jdx]);
                    last_sent_joint_mask.push_back(resp.joint_mask[idx].mask[jdx]);
                    jdx++;  
                }
            }

            if(idx < resp.group_type.size()) {
                group_type = resp.group_type[idx];
            }
            if(idx < resp.path_visualization_mode.size()) {
                path_visualization_mode = resp.path_visualization_mode[idx];
            }
            if(idx < resp.plan_on_move.size()) {
                plan_on_move = resp.plan_on_move[idx];
            }
            if(idx < resp.execute_on_plan.size()) {
                execute_on_plan = resp.execute_on_plan[idx];
            }
            if(idx < resp.plan_found.size()) {
                plan_found = resp.plan_found[idx];
            }

            for (auto& tol_mode: resp.tolerance) {
                if(tol_mode.mode == "Position Tolerance") {
                    position_tolerances.clear();
                    for (auto& tol_type: tol_mode.types) {
                        position_tolerances.push_back(tol_type);
                    }
                }
                if(tol_mode.mode == "Angle Tolerance") {
                    orientation_tolerances.clear();
                    for (auto& tol_type: tol_mode.types) {
                        orientation_tolerances.push_back(tol_type);
                    }
                }
            }
           
            if (resp.tolerance_setting.size() > 0) {
                for (auto& tol_info: resp.tolerance_setting[idx].tolerance_info) {
                    if(tol_info.mode == "Position Tolerance") {
                        position_tolerance = tol_info.types[0];
                    } 
                    if(tol_info.mode == "Angle Tolerance") {
                        orientation_tolerance = tol_info.types[0];
                    }
                }
            }

            
            stored_poses.clear();
            if(idx < resp.stored_pose_list.size()) {
                for (auto& stored_pose: resp.stored_pose_list[idx].data) {  
                    stored_poses.push_back(stored_pose);
                }
            }       
            setupDisplay(from);

        }   
    }

    return initialized;
}


void GroupControlsWidget::jointMaskChanged(QListWidgetItem* item) {

    ROS_INFO("GroupControlsWidget::jointMaskChanged()");       
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_JOINT_MAP;
    srv.request.group_name.push_back(group_name);

    nasa_robot_teleop::JointMask jm;
    for (int jdx=0; jdx<ui->joint_list->count(); jdx++) {
        if(item->text() != ui->joint_list->item(jdx)->text()) {
            jm.mask.push_back(joint_mask[jdx]);
            continue;
        }
        joint_mask[jdx] = (item->checkState()==Qt::Checked);
        jm.mask.push_back(joint_mask[jdx]);
    }
    srv.request.joint_mask.push_back(jm); 

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::jointMaskChanged() -- success");
        for (int jdx=0; jdx<ui->joint_list->count(); jdx++) {
            last_sent_joint_mask[jdx] = joint_mask[jdx];
        }
        setGroupDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::jointMaskChanged() -- failed to call service");
    }

}

void GroupControlsWidget::planOnMoveClicked(int d) {
    
    ROS_INFO("GroupControlsWidget::planOnMoveClicked()");    
    plan_on_move = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_PLAN_ON_MOVE;
    srv.request.group_name.push_back(group_name);
    srv.request.plan_on_move.push_back(plan_on_move);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::planOnMoveClicked() -- success");
        setGroupDataFromResponse(srv.response);

    }
    else
    {
        ROS_ERROR("GroupControlsWidget::planOnMoveClicked() -- failed to call service");
    }


}

void GroupControlsWidget::executeOnPlanClicked(int d) {
    
    ROS_INFO("GroupControlsWidget::executeOnPlanClicked()");    
    execute_on_plan = (d == Qt::Checked);
    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_EXECUTE_ON_PLAN;
    srv.request.group_name.push_back(group_name);
    srv.request.execute_on_plan.push_back(execute_on_plan);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::executeOnPlanClicked() -- success");
        setGroupDataFromResponse(srv.response);

    }
    else
    {
        ROS_ERROR("GroupControlsWidget::executeOnPlanClicked() -- failed to call service");
    }

}

bool GroupControlsWidget::positionToleranceChanged(const QString& text) {

    ROS_INFO("GroupControlsWidget::positionToleranceChanged()");    
    if(!initialized) {
        return true;
    }
    
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_TOLERANCES;
    srv.request.group_name.push_back(group_name);

    nasa_robot_teleop::ToleranceInfo pos_tol_info;
    pos_tol_info.mode = "Position Tolerance";
    pos_tol_info.types.push_back(ui->pos_tol->currentText().toStdString());
   
    srv.request.tolerance.push_back(pos_tol_info);
    position_tolerance = ui->pos_tol->currentText().toStdString();
    
    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::positionToleranceChanged() -- success");
        return true;//setGroupDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::positionToleranceChanged() -- failed to call service");
        return false;
    }
}
      
bool GroupControlsWidget::rotationToleranceChanged(const QString& text) {

    ROS_INFO("GroupControlsWidget::rotationToleranceChanged()");    
    if(!initialized) {
        return true;
    }

    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_TOLERANCES;
    srv.request.group_name.push_back(group_name);

    nasa_robot_teleop::ToleranceInfo rot_tol_info;
    rot_tol_info.mode = "Angle Tolerance";
    rot_tol_info.types.push_back(ui->rot_tol->currentText().toStdString());
    
    srv.request.tolerance.push_back(rot_tol_info);
    orientation_tolerance = ui->rot_tol->currentText().toStdString();
    
    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::rotationToleranceChanged() -- success");
        return true;//setGroupDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::rotationToleranceChanged() -- failed to call service");
        return false;
    }
}
          

bool GroupControlsWidget::planRequest() {

    ROS_INFO("GroupControlsWidget::planRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::PLAN_TO_MARKER;
    fillPlanRequest(srv);
    ROS_WARN("GroupControlsWidget: making PLAN_TO_MARKER call");    
    if (service_client_->call(srv)) {
        ROS_INFO("GroupControlsWidget::planRequest() -- success");
        return setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::planRequest() -- failed to call service");
        return false;
    }
}


void GroupControlsWidget::fillPlanRequest(nasa_robot_teleop::InteractiveControlsInterface &srv) {
    srv.request.group_name.push_back(group_name);
    std::string viz_type = ui->viz_type->currentText().toStdString();
    srv.request.path_visualization_mode.push_back(viz_type);
    srv.request.execute_on_plan.push_back(ui->execute_on_plan->checkState()==Qt::Checked);
    srv.request.group_type.push_back(group_type);

    nasa_robot_teleop::ToleranceInfo pos_tol_info;
    pos_tol_info.mode = "Position Tolerance";
    pos_tol_info.types.push_back(ui->pos_tol->currentText().toStdString());

    nasa_robot_teleop::ToleranceInfo rot_tol_info;
    rot_tol_info.mode = "Angle Tolerance";
    rot_tol_info.types.push_back(ui->rot_tol->currentText().toStdString());

    srv.request.tolerance.push_back(pos_tol_info);
    srv.request.tolerance.push_back(rot_tol_info);

    nasa_robot_teleop::JointMask jm;
    for (int jdx=0; jdx<ui->joint_list->count(); jdx++) {
        QListWidgetItem *item = ui->joint_list->item(jdx);       
        joint_mask[jdx] = (item->checkState()==Qt::Checked);
        last_sent_joint_mask[jdx] = joint_mask[jdx];
        jm.mask.push_back(joint_mask[jdx]);
    }
    srv.request.joint_mask.push_back(jm); 
}

bool GroupControlsWidget::executeRequest() {

    ROS_INFO("GroupControlsWidget::executeRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::EXECUTE_PLAN;
    srv.request.group_name.push_back(group_name);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::executeRequest() -- success");
        return setGroupDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::executeRequest() -- failed to call service");
        return false;
    }

}


bool GroupControlsWidget::toggleJointControlRequest() {

    ROS_INFO("GroupControlsWidget::toggleJointControlRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::TOGGLE_POSTURE_CONTROLS;
    srv.request.group_name.push_back(group_name);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::toggleJointControlRequest() -- success");
        return setGroupDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::toggleJointControlRequest() -- failed to call service");
        return false;
    }
}

void GroupControlsWidget::setToolOffsetClicked() {
    ROS_INFO("GroupControlsWidget::setToolOffsetClicked()");    
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::SET_TOOL_OFFSET;
    srv.request.group_name.push_back(group_name);
    if (service_client_->call(srv)) {
        ROS_INFO("GroupControlsWidget::setToolOffsetClicked() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::setToolOffsetClicked() -- failed to call service");
    }
}

void GroupControlsWidget::clearToolOffsetClicked() {
    ROS_INFO("GroupControlsWidget::clearToolOffsetClicked()");    
    nasa_robot_teleop::InteractiveControlsInterface srv;
    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::CLEAR_TOOL_OFFSET;
    srv.request.group_name.push_back(group_name);
    if (service_client_->call(srv)) {
        ROS_INFO("GroupControlsWidget::clearToolOffsetClicked() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::clearToolOffsetClicked() -- failed to call service");
    }
}

bool GroupControlsWidget::storedPoseRequest() {

    ROS_INFO("GroupControlsWidget::storedPoseRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::EXECUTE_STORED_POSE;
    srv.request.group_name.push_back(group_name);
    std::string stored_pose = ui->stored_pose_list->currentText().toStdString();
    srv.request.stored_pose_name.push_back(stored_pose);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::storedPoseRequest() -- success");
        setGroupDataFromResponse(srv.response);
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::storedPoseRequest() -- failed to call service");
        return false;
    }

    return true;
}

