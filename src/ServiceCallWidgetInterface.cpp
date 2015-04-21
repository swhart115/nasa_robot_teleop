/**
 * Put copyright notice here
 */
#include "ServiceCallWidgetInterface.hpp"

namespace rviz_interactive_controls_panel {

    ServiceCallWidgetInterface(QWidget *parent) : QWidget(parent) { }
    
    void ServiceCallWidgetInterface::updateFromResponse(
            nasa_robot_teleop::InteractiveControlsInterfaceResponse &rsp) {
        // empty base method
    }
    
}
