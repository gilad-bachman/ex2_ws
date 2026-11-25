/*
 * Exercise 2 - ID: 211397054
 *
 * The robot's actuators, behind one small class. The leaves move the robot
 * and light the LEDs through it; it gets the actuators as parameters.
 * On top of the raw actuators it has the movement helpers (Stop, DriveTo, FollowWall).
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

      /* ---- The raw actuators ---- */

      /* Wheels: the speed of each wheel, in m/s (clamped to MAX_WHEEL_SPEED) */
      void SetWheels(Real f_left, Real f_right);

      /* LEDs: light them in one color */
      void SetLEDs(const CColor& c_color);

      /* ---- Movement helpers ---- */

      /* Stand still */
      void Stop();

      /*
       * One step towards c_target (world frame), at up to f_speed m/s.
       * Fusion of two votes, in the robot's frame:
       *    pull: towards the target, always the same strength
       *    push: away from every rangefinder that sees something close
       * then P control on the heading of their sum.
       * It only steers: the leaf decides when it has arrived, or failed.
       */
      void DriveTo(const SBlackboard& s_board, const CVector2& c_target, Real f_speed);

      /*
       * One step along a wall, keeping it on the right (b_right) or the left,
       * at up to f_speed m/s. PD control on the distance to the wall
       * (Tutorial 4); f_prev_error is the PD's memory, kept by the caller.
       * Turns in place at an inside corner, curves around an outside one.
       */
      void FollowWall(const SBlackboard& s_board, bool b_right, Real& f_prev_error, Real f_speed);

      /* Where rangefinder un_index (0..7) looks, in the robot's frame (0 = front, left positive) */
      static CRadians RangefinderAngle(size_t un_index);

   private:

      /* P control on a heading error (robot frame, [-pi, pi]) */
      void Steer(const CRadians& c_error, Real f_speed);

      CCI_PiPuckDifferentialDriveActuator* m_pcWheels;
      CCI_PiPuckColorLEDsActuator* m_pcLEDs;
   };

}

#endif
