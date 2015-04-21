/**
 * Put copyright notice here
 */
#ifndef SERVICE_CALL_WIDGET_INTERFACE_HPP
#define SERVICE_CALL_WIDGET_INTERFACE_HPP

#include "nasa_robot_teleop/InteractiveControlsInterface.h"

namespace rviz_interactive_controls_panel {

    /** Provide a multiple inheritance interface for
     * RVizInteractiveControlsPanel widgets to receive service call
     * responses. */
    class ServiceCallWidgetInterface {
        public:
            virtual void updateFromResponse(nasa_robot_teleop::InteractiveControlsInterfaceResponse &rsp) = 0;
    };
}
#endif

