/*
 * Exercise 2 - ID: 211397054
 *
 * DriveTo follows Tutorial 4 (go to any point: P control on the heading) and
 * Tutorial 5 (fusion: a pull to the goal plus a push from every obstacle).
 */
#include "actuators.hpp"

#include <argos3/core/utility/math/angles.h>
#include <algorithm>
#include <cmath>

namespace argos {

   namespace {

      /* Wheels */
      const Real MAX_WHEEL_SPEED = CCI_PiPuckDifferentialDriveActuator::MAX_WHEEL_VELOCITY;   // m/s (0.154)

      /* P control on the heading (Tutorial 4) */
      const Real KP = 0.1;                 // m/s of wheel difference per radian of error
      const Real TURN_IN_PLACE = 0.5;      // rad: farther off than this, turn without driving

      /* The pull of the target and the push of the obstacles (Tutorial 5) */
      const Real PULL_GAIN = 0.1;          // the pull, always this strong
      const Real PUSH_GAIN = 0.2;          // the push of one rangefinder, at contact
      const Real PUSH_RANGE = 0.10;        // m: farther than this, no push (rangefinders see 0.2)

      /*
       * Where each rangefinder looks, in the robot's frame (0 = front, left positive).
       * From the sensors' positions on the robot, in ARGoS's
       * ci_pipuck_rangefinders_sensor.cpp (MAP_SENSOR_CONFIG): 0..3 on the right, 4..7 on the left.
       */
      const Real RANGEFINDER_DEG[8] = { -16.1, -48.4, -90.0, -142.9, 142.9, 90.0, 48.4, 16.1 };

      /* Wall following (Tutorial 4's wall follower: PD on the distance to the wall) */
      const Real WALL_DISTANCE = 0.07;     // m: keep the wall this far away
      const Real WALL_LOST = 0.15;         // m: the side sensors see nothing closer: the wall has ended
      const Real WALL_AHEAD = 0.07;        // m: a wall this close in front: turn away in place
      const Real WALL_KP = 1.0;            // m/s of wheel difference per m of error
      const Real WALL_KD = 5.0;            // ... per m of change in the error, per step
      const Real WALL_CORNER_TURN = 0.02;  // m/s: lost the wall, curve back around its end (radius ~14 cm at 0.1 m/s)
      const Real WALL_SPIN = 0.06;         // m/s: turning in place, away from a wall ahead
   }

   CRadians CActuators::RangefinderAngle(size_t un_index) {
      return ToRadians(CDegrees(RANGEFINDER_DEG[un_index]));
   }

   CActuators::CActuators(CCI_PiPuckDifferentialDriveActuator* pc_wheels,
                          CCI_PiPuckColorLEDsActuator* pc_leds)
      : m_pcWheels(pc_wheels), m_pcLEDs(pc_leds) {}

   void CActuators::SetWheels(Real f_left, Real f_right) {
      /* Faster than the motors can go? Scale both wheels down together, so the
         robot still turns the same way (clamping each one alone would lose the turn) */
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
      /* The pull: towards the target, turned into the robot's frame */
      CVector2 cPull = c_target - s_board.Position;
      if (cPull.Length() > 0.0) cPull.Normalize();
      cPull.Rotate(-s_board.Heading);
      cPull *= PULL_GAIN;

      /* The push: away from what each rangefinder sees, stronger when closer */
      CVector2 cPush;
      for (size_t i = 0; i < s_board.Proximity.size(); ++i) {
         Real fDist = s_board.Proximity[i];
         if (fDist >= PUSH_RANGE) continue;
         Real fStrength = PUSH_GAIN * (PUSH_RANGE - fDist) / PUSH_RANGE;
         cPush += CVector2(fStrength, RangefinderAngle(i) + CRadians::PI);
      }

      /* Fusion: add the votes, and steer where their sum points */
      CVector2 cSum = cPull + cPush;
      Steer(cSum.Angle().SignedNormalize(), f_speed);
   }

   void CActuators::FollowWall(const SBlackboard& s_board, bool b_right, Real& f_prev_error, Real f_speed) {
      /* Which sensors: the front pair, and the side the wall is on (side + front-side) */
      Real fAhead = std::min(s_board.Proximity[0], s_board.Proximity[7]);
      Real fSide = b_right ? std::min(s_board.Proximity[1], s_board.Proximity[2])
                           : std::min(s_board.Proximity[5], s_board.Proximity[6]);
      /* u > 0 turns left. Away from a wall on the right is left: +1. On the left: -1 */
      Real fAway = b_right ? 1.0 : -1.0;

      /* A wall ahead (an inside corner): turn away from it, in place */
      if (fAhead < WALL_AHEAD) {
         f_prev_error = 0.0;
         SetWheels(-fAway * WALL_SPIN, fAway * WALL_SPIN);
         return;
      }

      /* The wall has ended (an outside corner): curve towards where it was */
      if (fSide >= WALL_LOST) {
         f_prev_error = 0.0;
         Real fU = -fAway * WALL_CORNER_TURN;
         SetWheels(f_speed - fU, f_speed + fU);
         return;
      }

      /* Along the wall: PD on the distance. Too close (error > 0): turn away */
      Real fError = WALL_DISTANCE - fSide;
      Real fU = fAway * (WALL_KP * fError + WALL_KD * (fError - f_prev_error));
      f_prev_error = fError;
      SetWheels(f_speed - fU, f_speed + fU);
   }

   void CActuators::Steer(const CRadians& c_error, Real f_speed) {
      Real fError = c_error.GetValue();
      Real fU = KP * fError;                                     // u > 0 turns left
      Real fV = (std::abs(fError) < TURN_IN_PLACE) ? f_speed : 0.0;
      SetWheels(fV - fU, fV + fU);
   }

}
