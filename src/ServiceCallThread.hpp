/**
 * Put copyright notice here
 */
#ifndef SERVICE_CALL_THREAD_HPP
#define SERVICE_CALL_THREAD_HPP

#include <ros/ros.h>
#include "nasa_robot_teleop/InteractiveControlsInterface.h"
#include "nasa_robot_teleop/ServiceCallWidgetInterface.h"

namespace rviz_interactive_controls_panel {
    
    // typedefs for signals/slots
    typedef nasa_robot_teleop::InteractiveControlsInterface ICIface;
    //typedef nasa_robot_teleop::InteractiveControlsInterfaceRequest ICIfaceReq;
    //typedef nasa_robot_teleop::InteractiveControlsInterfaceResponse ICIfaceRsp;
    
    class ServiceCallThread: public QObject {
        Q_OBJECT
        public:
            ServiceCallThread(double frequency);
            bool addCaller(const std::string &, ServiceCallWidgetInterface*);
            bool removeCaller(const std::string &);
		public Q_SLOTS:
			void start();
			void abort();
			void newCall(ICIface);
			void clearCall();
		Q_SIGNALS:
			void callResult(int, bool);
			void callAbort(int);
			void callFinished();
			void serviceThreadStillAlive();
			void sendingServiceCall(int);
			void finishedServiceCall(bool);
		protected:
			void timerEvent(QTimerEvent *te);
		private:
			double m_frequency;
			bool m_signal;
			
			bool heartbeat_on;
			
			std::deque<GenericService> commandQueue;
			
			QBasicTimer m_timer;
			QMutex m_callMutex;
			unsigned char m_counter;
			ros::WallTime m_last_time;
			ros::ServiceClient gsc_client;
	};
	

