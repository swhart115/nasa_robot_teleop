/**
 * Put copyright notice here
 */
#ifndef SERVICE_CALL_WIDGET_INTERFACE_HPP
#define SERVICE_CALL_WIDGET_INTERFACE_HPP

#include "nasa_robot_teleop/InteractiveControlsInterface.h"
#include <QString>
#include <QWidget>

namespace rviz_interactive_controls_panel {

    /** Provide a base class interface for RVizInteractiveControlsPanel
     * widgets to send \c nasa_robot_teleop::InteractiveControlsInterface
     * service calls as a signal and receive the call's response.
     *
     * Subclasses are expected to be \c QWidgets (likely placed into
     * a \c QTabWidget). When instantiated, subclasses should be
     * 'registered' with an instance of \c ServiceCallThread using a
     * unique tag and a pointer to the subclass. The thread object will
     * \c connect the subclass's \c sendCall signal to its \c addCall
     * slot; the tag and pointer allows the thread to direct the call's
     * response to the appropriate object.
     * 
     * When making a service call, the message is constructed and relayed
     * as a signal; the call will be queued in the thread and control will
     * return. The thread will attempt to complete the service call until
     * it either succeeds or the subclass instance relays a new message,
     * which supercedes the old message.
     * 
     * Upon service call success, the thread will execute the subclass's
     * \c updateFromResponse method. */
    class ServiceCallWidgetInterface : public QWidget {
        Q_OBJECT
        public:
            typedef nasa_robot_teleop::InteractiveControlsInterface ICIface;
            
            ServiceCallWidgetInterface(QWidget *parent = 0);
            ~ServiceCallWidgetInterface();
            QString getTag();
            virtual void updateFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &rsp);
        
        Q_SIGNALS:
            void callerRemoval(QString);
            void sendCall(QString, ICIface);
        
        protected:
            QString m_tag;
        
        private:
            static int TAG_COUNT;

    };
}
#endif

