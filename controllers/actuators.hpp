/*
 * Exercise 2 - ID: 211397054
 *
 * The wheels and LEDs, and the movement helpers the leaves use.
 */
#ifndef ACTUATORS_HPP
#define ACTUATORS_HPP

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>

#include "blackboard.hpp"

namespace argos {

   class CActuators {

   public:

      CActuators(CCI_PiPuckDifferentialDriveActuator* pc_wheels,
                 CCI_PiPuckColorLEDsActuator* pc_leds);

      /* Wheel speeds in m/s */
      void SetWheels(Real f_left, Real f_right);
      void SetLEDs(const CColor& c_color);
      void Stop();

      /* One step towards c_target: P control on the heading of pull + obstacle push (Tutorials 4, 5) */
      void DriveTo(const SBlackboard& s_board, const CVector2& c_target, Real f_speed);

      /* One step along a wall on the right or left: PD on the distance (Tutorial 4) */
      void FollowWall(const SBlackboard& s_board, bool b_right, Real& f_prev_error, Real f_speed);

      /* Direction of rangefinder un_index in the robot's frame (0 = front, left positive) */
      static CRadians RangefinderAngle(size_t un_index);

   private:

      void Steer(const CRadians& c_error, Real f_speed);

      CCI_PiPuckDifferentialDriveActuator* m_pcWheels;
      CCI_PiPuckColorLEDsActuator* m_pcLEDs;
   };

}

#endif
