/*
 * Exercise 2 - ID: <your ID here>
 */
#ifndef CONTROLLER2_HPP
#define CONTROLLER2_HPP

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

#include "battery.hpp"
#include "light_order.hpp"

namespace argos {

   class Controller2 : public CCI_Controller, public CBatteryEquipped {

   public:

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

   private:

      /* Actuators */
      CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;

      /* Sensors */
      CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
      CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
      CCI_PositioningSensor* m_pcPositioning = nullptr;
      CCI_PiPuckSystemSensor* m_pcSystem = nullptr;

      /* The lights, in the order to visit them (sensors/light_order.hpp) */
      std::vector<SLightInfo> m_vecOrder;
   };
}

#endif
