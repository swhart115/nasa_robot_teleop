/**
 * Put copyright notice here
 */
#include "ServiceCallThread.hpp"

namespace rviz_interactive_controls_panel {
    
    // TODO: this is cut & pasted from DRC UI (drc_wall_ui.cpp) without
    //   modification! Serves as a start only, needs adjustment!
    ServiceCallThread::ServiceCallThread(double frequency)
        : m_frequency(frequency)
        , m_topic("/interactive_control/configure")
    {
        ros::NodeHandle nh;
        m_counter = 0;
        heartbeat_on=false;
        ROS_WARN("ServiceCallThread: setting service client [%s]", m_topic.c_str());
        //m_client = nh.serviceClient<nasa_robot_teleop::InteractiveControlsInterface>(topic, true);
        m_client = nh.serviceClient<ICIface>(m_topic, true);
        ROS_WARN("ServiceCallThread: set up service client [%s]", m_topic.c_str());
    }
	
    bool ServiceCallThread::addCaller(ServiceCallWidgetInterface* caller) {
        bool retval = false;
        if (connect(caller, SIGNAL(sendCall(QString, ICIface)),
                    this, SLOT(newCall(QString, ICIface)))) {
            connect(caller, SIGNAL(callerRemoval(QString)),
                    this, SLOT(removeCaller(QString)));
            m_callers[caller->getTag()] = caller;
            retval = true;
        }
        return retval;
    }
	
    void ServiceCallThread::removeCaller(QString tag) {
        if (m_callers.count(tag) > 0) {
            m_callers.erase(tag);
        }
    }
    
    void ServiceCallThread::newCall(QString tag, ICIface srv) {
        QMutexLocker clilock(&m_callMutex);
        ROS_DEBUG("ServiceCallThread: Queue [%s] call @ position %d",
             InteractiveControlsInterfaceUtils::actionStr(srv.request.action_type).c_str(), (int)callQueue.size());
        callQueue.push_back(srv);
    }
    
    void ServiceCallThread::clearCall() {
        QMutexLocker clilock(&m_callMutex);
        ROS_WARN("Clearing call queue");
        callQueue.clear();
    }
    
    void ServiceCallThread::abort() {
        m_timer.stop();
        Q_EMIT callFinished();
        ROS_DEBUG("ServiceCallThread: abort/end...");
    }
    
    void ServiceCallThread::start() {
        ROS_DEBUG("ServiceCallThread: starting...");
        double inner_freq = 10.0; //Need to paramterize
        m_timer.start((1000 / inner_freq), this);
    }
    
    void ServiceCallThread::timerEvent(QTimerEvent *te) {
        //drc_msgs::GenericServiceCall srv;
		//srv.request.type = service_manager::HEARTBEAT;
		//
        nasa_robot_teleop::InteractiveControlsInterface srv;
        bool gui_command = false;
        {
            QMutexLocker clilock(&m_callMutex);
            if (!callQueue.empty()) {
                gui_command = true;
                srv = callQueue[0];
                callQueue.pop_front();
            }
        }
        
        bool result;
        ros::WallTime now = ros::WallTime::now();
        
        if (gui_command ||
            (heartbeat_on &&
             now >= m_last_time + ros::WallDuration(1 / m_frequency))) {
            if (m_client) {
                //srv.request.counter = m_counter;
                m_counter = (m_counter + 1) % 256;
                m_last_time = now;
                //std::string srvName = service_manager::getServiceTypeName(srv.request.type);
                // ROS_INFO("Calling queued service of type %s", drc_msgs::getServiceTypeName(srv.request.type).c_str());
                if (gui_command) {
                    ROS_DEBUG("ServiceCallThread: emit 'sendingServiceCall'");
                    Q_EMIT sendingServiceCall(srv.request.action_type);
                }
                ROS_DEBUG("ServiceCallThread: making [%s] service call",
                        InteractiveControlsInterfaceUtils::actionStr(srv.request.action_type).c_str());
                result = m_client.call(srv);
                if (gui_command) {
                    ROS_DEBUG("ServiceCallThread: emit 'finishedServiceCall'");
                    Q_EMIT finishedServiceCall(result);
                }
                
                if (result) {
                    //if (srv.request.type == service_manager::STOP_HEARTBEAT) {
                    //    heartbeat_on=false;
                    //} else if (srv.request.type == service_manager::HEARTBEAT) {
                    //    heartbeat_on=true;
                    //}
                }
            } else {
                ROS_ERROR_THROTTLE(1, "ServiceCallThread: service not found!!");
                ros::NodeHandle nh;
                ROS_WARN("ServiceCallThread: timer, setting up service client [%s", m_topic.c_str());
                m_client = nh.serviceClient<nasa_robot_teleop::InteractiveControlsInterface>(m_topic, true);
                ROS_WARN("ServiceCallThread: timer, set up service client [%s]", m_topic.c_str());
                result = false;
            }
            
            if (result) {
                Q_EMIT serviceThreadStillAlive();
            }
            Q_EMIT callResult(srv.request.action_type, result);
        }
    }

}

