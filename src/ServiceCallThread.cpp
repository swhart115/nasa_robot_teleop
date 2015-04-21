/**
 * Put copyright notice here
 */
#include "ServiceCallThread.hpp"

namespace rviz_interactive_controls_panel {
    
    // TODO: this is cut & pasted from DRC UI (drc_wall_ui.cpp) without
    //   modification! Serves as a start only, needs adjustment!
  ServiceCallThread::ServiceCallThread(double frequency) {
    ros::NodeHandle nh;
    m_frequency = frequency;
    m_counter = 0;
    heartbeat_on=false;
    ROS_WARN("ServiceCallThread: setting up service client [/generic_service_call]");
    gsc_client = nh.serviceClient<drc_msgs::GenericServiceCall>("/generic_service_call", true);
    ROS_WARN("ServiceCallThread: set up service client [/generic_service_call]");
  }
	
	void ServiceCallThread::newCommand(GenericService srv) {
		QMutexLocker clilock(&m_callMutex);
		ROS_DEBUG("ServiceCallThread: Queue [%s] call @ position %d",
		         service_manager::getServiceTypeName(srv.request.type).c_str(),
					(int)commandQueue.size());
		commandQueue.push_back(srv);
	}
	
  void ServiceCallThread::clearCommand() {
    QMutexLocker clilock(&m_callMutex);
    ROS_WARN("Clearing command queue");
    commandQueue.clear();
  }
	
  void ServiceCallThread::abort() {
    m_timer.stop();
    Q_EMIT callFinished();
    ROS_DEBUG("MainWindow::ServiceCallThread: abort/end...");
  }
	
  void ServiceCallThread::start() {
    ROS_DEBUG("MainWindow::ServiceCallThread: starting...");
    double inner_freq = 10.0; //Need to paramterize
    m_timer.start((1000 / inner_freq), this);
  }
	
	void ServiceCallThread::timerEvent(QTimerEvent *te) {
		drc_msgs::GenericServiceCall srv;
		srv.request.type = service_manager::HEARTBEAT;
		
		bool gui_command = false;
		{
			QMutexLocker clilock(&m_callMutex);
			if (!commandQueue.empty()) {
				gui_command = true;
				srv = commandQueue[0];
				commandQueue.pop_front();
			}
		}
		
		bool result;
		ros::WallTime now = ros::WallTime::now();
		
		if (gui_command || (heartbeat_on && now >= m_last_time + ros::WallDuration(1 / m_frequency))) {
			if (gsc_client) {
				srv.request.counter = m_counter;
				m_counter = (m_counter + 1) % 256;
				m_last_time = now;
				//std::string srvName = service_manager::getServiceTypeName(srv.request.type);
				// ROS_INFO("Calling queued service of type %s", drc_msgs::getServiceTypeName(srv.request.type).c_str());
				if (gui_command) {
					ROS_DEBUG("ServiceCallThread: emit 'sendingServiceCall'");
					Q_EMIT sendingServiceCall(srv.request.type);
				}
				ROS_DEBUG("ServiceCallThread: making [%s] service call",
				         service_manager::getServiceTypeName(srv.request.type).c_str());
				result = gsc_client.call(srv);
				if (gui_command) {
					ROS_DEBUG("ServiceCallThread: emit 'finishedServiceCall'");
					Q_EMIT finishedServiceCall(result);
				}
				
				if (result) {
					if (srv.request.type == service_manager::STOP_HEARTBEAT) {
						heartbeat_on=false;
					} else if (srv.request.type == service_manager::HEARTBEAT) {
						heartbeat_on=true;
					}
				}
			} else {
				ROS_ERROR_THROTTLE(1, "Generic service not found!!");
				ros::NodeHandle nh;
				ROS_WARN("ServiceCallThread: timer, setting up service client [/generic_service_call]");
				gsc_client = nh.serviceClient<drc_msgs::GenericServiceCall>("/generic_service_call", true);
				ROS_WARN("ServiceCallThread: timer, set up service client [/generic_service_call]");
				result = false;
			}
			
			if (result) {
				Q_EMIT serviceThreadStillAlive();
			}
			Q_EMIT callResult(srv.request.type, result);
		}
	}

}

