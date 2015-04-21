/**
 * Put copyright notice here
 */
#include "ServiceCallWidgetInterface.hpp"

namespace rviz_interactive_controls_panel {
    
    // set the static counter for gensym tags
    int ServiceCallWidgetInterface::TAG_COUNT = 0;
    
    ServiceCallWidgetInterface::ServiceCallWidgetInterface(QWidget *parent)
        : QWidget(parent)
    {
        int count = ServiceCallWidgetInterface::TAG_COUNT++;
        m_tag = "caller-";
        m_tag += QString::number(count);
        setObjectName(m_tag);
    }
    
    ServiceCallWidgetInterface::~ServiceCallWidgetInterface() {
        // TODO: not clear to me this'll work; will object be destroyed
        //   before signal can be processed?
        Q_EMIT callerRemoval(m_tag);
    }
    
    void ServiceCallWidgetInterface::getTag() {
        return m_tag;
    }
    
    void ServiceCallWidgetInterface::updateFromResponse(
            nasa_robot_teleop::InteractiveControlsInterfaceResponse &rsp) {
        // empty base method
    }
    
}
