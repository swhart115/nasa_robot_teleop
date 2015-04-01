#include "GroupControlsWidget.hpp"

using namespace rviz_interactive_controls_panel;
using namespace std;

GroupControlsWidget::GroupControlsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GroupControls)
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

}

void GroupControlsWidget::setupDisplay() {

	ui->joint_list->clear();
	
	ui->type_label->setText(QString(group_type.c_str()));

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
		ui->plan_label->setText(QString("PLAN FOUND"));
	} else {
		ui->plan_label->setText(QString("NO PLAN"));
	}

	if(group_type=="cartesian") {
		if(plan_on_move) {
	        ui->plan_on_move->setCheckState(Qt::Checked);
	    } else {
	        ui->plan_on_move->setCheckState(Qt::Unchecked);
	    }

	    ui->pos_tol->clear();
	    for (auto& pt: position_tolerances) {
			ui->pos_tol->addItem(QString(pt.c_str()));
		}
			
		ui->rot_tol->clear();
	    for (auto& rt: orientation_tolerances) {
			ui->rot_tol->addItem(QString(rt.c_str()));
		}

		int index;
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

    if(execute_on_plan) {
        ui->execute_on_plan->setCheckState(Qt::Checked);
    } else {
        ui->execute_on_plan->setCheckState(Qt::Unchecked);
    }

    
	
}


bool GroupControlsWidget::planRequest() {

	ROS_INFO("GroupControlsWidget::planRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::PLAN_TO_MARKER;
    srv.request.group_name.push_back(group_name);
  	std::string viz_type = ui->viz_type->currentText().toStdString();
    srv.request.path_visualization_mode.push_back(viz_type);
    srv.request.execute_on_plan.push_back(ui->execute_on_plan->checkState()==Qt::Checked);

    nasa_robot_teleop::ToleranceInfo pos_tol_info;
    pos_tol_info.mode = "Position Tolerance";
    pos_tol_info.types.push_back(ui->pos_tol->currentText().toStdString());

    nasa_robot_teleop::ToleranceInfo rot_tol_info;
    rot_tol_info.mode = "Angle Tolerance";
    rot_tol_info.types.push_back(ui->rot_tol->currentText().toStdString());

    srv.request.tolerance.push_back(pos_tol_info);
    srv.request.tolerance.push_back(rot_tol_info);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::planRequest() -- success");
        setupDisplay();
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::planRequest() -- failed to call service");
        return false;
    }

    return true;
}


bool GroupControlsWidget::executeRequest() {

	ROS_INFO("GroupControlsWidget::executeRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::EXECUTE_PLAN;
    srv.request.group_name.push_back(group_name);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::executeRequest() -- success");
        setupDisplay();

    }
    else
    {
        ROS_ERROR("GroupControlsWidget::executeRequest() -- failed to call service");
        return false;
    }

    return true;
}


bool GroupControlsWidget::toggleJointControlRequest() {

	ROS_INFO("GroupControlsWidget::toggleJointControlRequest()");    

    nasa_robot_teleop::InteractiveControlsInterface srv;

    srv.request.action_type = nasa_robot_teleop::InteractiveControlsInterfaceRequest::TOGGLE_POSTURE_CONTROLS;
    srv.request.group_name.push_back(group_name);

    if (service_client_->call(srv))
    {
        ROS_INFO("GroupControlsWidget::toggleJointControlRequest() -- failed to call service");
        setupDisplay();
    }
    else
    {
        ROS_ERROR("GroupControlsWidget::toggleJointControlRequest() -- failed to call service");
        return false;
    }

    return true;
}
