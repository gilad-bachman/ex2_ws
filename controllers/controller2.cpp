/*
 * Exercise 2 - ID: <your ID here>
 *
 * The starting point: every sensor and actuator is set up, the referee keeps
 * the battery up to date, and the robot stands still. Replace ControlStep() with
 * your executive: tick your behavior tree once per step.
 */
#include "controller2.hpp"

namespace argos {

   void Controller2::Init(TConfigurationNode& t_tree) {
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera->Enable();
      m_vecOrder = ReadLightOrder(t_tree);

      /* Your code here */

   }

   void Controller2::ControlStep() {
      /* Your code here */
   }

   REGISTER_CONTROLLER(Controller2, "controller2");

}
