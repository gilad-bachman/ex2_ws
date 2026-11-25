/*
 * Exercise 2 - ID: 211397054
 */
#include "actuators.hpp"

#include <argos3/core/utility/math/angles.h>
#include <algorithm>
#include <cmath>

namespace argos {

   namespace {

      const Real MAX_WHEEL_SPEED = CCI_PiPuckDifferentialDriveActuator::MAX_WHEEL_VELOCITY;

      /* Go to a point */
      const Real KP = 0.1;
      const Real TURN_IN_PLACE = 0.5;   // rad
      const Real PULL_GAIN = 0.1;
      const Real PUSH_GAIN = 0.2;
      const Real PUSH_RANGE = 0.10;     // m

      /* From the sensor positions in ARGoS's ci_pipuck_rangefinders_sensor.cpp */
      const Real RANGEFINDER_DEG[8] = { -16.1, -48.4, -90.0, -142.9, 142.9, 90.0, 48.4, 16.1 };

      /* Wall following (distances in m, speeds in m/s) */
      const Real WALL_DISTANCE = 0.07;
      const Real WALL_LOST = 0.15;
      const Real WALL_AHEAD = 0.07;
      const Real WALL_KP = 1.0;
      const Real WALL_KD = 5.0;
      const Real WALL_CORNER_TURN = 0.02;
      const Real WALL_SPIN = 0.06;
   }

   CRadians CActuators::RangefinderAngle(size_t un_index) {
      return ToRadians(CDegrees(RANGEFINDER_DEG[un_index]));
   }

   CActuators::CActuators(CCI_PiPuckDifferentialDriveActuator* pc_wheels,
                          CCI_PiPuckColorLEDsActuator* pc_leds)
      : m_pcWheels(pc_wheels), m_pcLEDs(pc_leds) {}

   void CActuators::SetWheels(Real f_left, Real f_right) {
      /* Scale both wheels together, so the turn is kept (clamping one would lose it) */
      Real fFastest = std::max(std::abs(f_left), std::abs(f_right));
      if (fFastest > MAX_WHEEL_SPEED) {
         f_left *= MAX_WHEEL_SPEED / fFastest;
         f_right *= MAX_WHEEL_SPEED / fFastest;
      }
      m_pcWheels->SetLinearVelocity(f_left, f_right);
   }

   void CActuators::SetLEDs(const CColor& c_color) {
      m_pcLEDs->SetRingLEDs(c_color);
   }

   void CActuators::Stop() {
      SetWheels(0.0, 0.0);
   }

   void CActuators::DriveTo(const SBlackboard& s_board, const CVector2& c_target, Real f_speed) {
      CVector2 cPull = c_target - s_board.Position;
      if (cPull.Length() > 0.0) cPull.Normalize();
      cPull.Rotate(-s_board.Heading);
      cPull *= PULL_GAIN;

      CVector2 cPush;
      for (size_t i = 0; i < s_board.Proximity.size(); ++i) {
         Real fDist = s_board.Proximity[i];
         if (fDist >= PUSH_RANGE) continue;
         Real fStrength = PUSH_GAIN * (PUSH_RANGE - fDist) / PUSH_RANGE;
         cPush += CVector2(fStrength, RangefinderAngle(i) + CRadians::PI);
      }

      Steer((cPull + cPush).Angle().SignedNormalize(), f_speed);
   }

   void CActuators::FollowWall(const SBlackboard& s_board, bool b_right, Real& f_prev_error, Real f_speed) {
      Real fAhead = std::min(s_board.Proximity[0], s_board.Proximity[7]);
      Real fSide = b_right ? std::min(s_board.Proximity[1], s_board.Proximity[2])
                           : std::min(s_board.Proximity[5], s_board.Proximity[6]);
      /* u > 0 turns left, so away from a right-hand wall is +1 */
      Real fAway = b_right ? 1.0 : -1.0;

      /* Inside corner: turn away in place */
      if (fAhead < WALL_AHEAD) {
         f_prev_error = 0.0;
         SetWheels(-fAway * WALL_SPIN, fAway * WALL_SPIN);
         return;
      }

      /* Outside corner: curve around the end of the wall */
      if (fSide >= WALL_LOST) {
         f_prev_error = 0.0;
         Real fU = -fAway * WALL_CORNER_TURN;
         SetWheels(f_speed - fU, f_speed + fU);
         return;
      }

      Real fError = WALL_DISTANCE - fSide;
      Real fU = fAway * (WALL_KP * fError + WALL_KD * (fError - f_prev_error));
      f_prev_error = fError;
      SetWheels(f_speed - fU, f_speed + fU);
   }

   void CActuators::Steer(const CRadians& c_error, Real f_speed) {
      Real fError = c_error.GetValue();
      Real fU = KP * fError;
      Real fV = (std::abs(fError) < TURN_IN_PLACE) ? f_speed : 0.0;
      SetWheels(fV - fU, fV + fU);
   }

}
