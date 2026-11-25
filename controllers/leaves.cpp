/*
 * Exercise 2 - ID: 211397054
 */
#include "leaves.hpp"
#include "battery.hpp"

#include <algorithm>
#include <cmath>

namespace argos {

   namespace {

      /* Distances in m, times in s */
      const Real SPEED = CCI_PiPuckDifferentialDriveActuator::MAX_WHEEL_VELOCITY;
      const Real ARRIVED = 0.05;
      const Real PROGRESS = 0.05;
      const Real TIMEOUT = 5.0;

      /* Bug2 */
      const Real FOLLOW_SPEED = 0.1;
      const Real LEAVE_GAIN = 0.05;
      const Real MLINE_TOLERANCE = 0.04;
      const Real HIT_POINT_LEFT = 0.15;
      const Real HIT_POINT_BACK = 0.08;
      const Real FOLLOW_TIMEOUT = 20.0;

      const Real SEARCH_LEG = 5.0;
      const Real VISIT_TIME = 2.5;        // the referee needs 2 s
      const Real VISIT_RADIUS = 0.12;     // the referee allows 0.15

      /*
       * Battery threshold: the straight trip home costs DRAIN / SPEED (about
       * 5.2 % per m); the real way is longer, so allow twice that plus a reserve.
       */
      const Real HOME_SAFETY = 2.0;
      const Real BATTERY_RESERVE = 5.0;   // %
      const Real CHARGER_RADIUS = 0.12;
      const Real FULL = 99.0;             // %

      const CColor PINK(255, 105, 180);
   }

   EStatus CIsBatteryLow::Tick() {
      m_cActuators.SetLEDs(CColor::GRAY80);
      Real fDistance = (m_sBoard.Home - m_sBoard.Position).Length();
      Real fTripCost = CBattery::DRAIN_PER_SECOND * fDistance / SPEED;
      Real fThreshold = HOME_SAFETY * fTripCost + BATTERY_RESERVE;
      return (m_sBoard.BatteryLevel < fThreshold) ? EStatus::SUCCESS : EStatus::FAILURE;
   }

   EStatus CCharge::Tick() {
      m_cActuators.SetLEDs(CColor::PURPLE);
      m_cActuators.Stop();

      if ((m_sBoard.Home - m_sBoard.Position).Length() > CHARGER_RADIUS)
         return EStatus::FAILURE;
      if (m_sBoard.BatteryLevel >= FULL)
         return EStatus::SUCCESS;
      return EStatus::RUNNING;
   }

   EStatus CStayHome::Tick() {
      m_cActuators.SetLEDs(CColor::BROWN);
      m_cActuators.Stop();

      if (!m_bStarted) {
         m_bStarted = true;
         m_fEndTime = m_sBoard.Time + VISIT_TIME;
      }

      if ((m_sBoard.Home - m_sBoard.Position).Length() > CHARGER_RADIUS) {
         m_bStarted = false;
         return EStatus::FAILURE;
      }

      if (m_sBoard.Time >= m_fEndTime) {
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      return EStatus::RUNNING;
   }

   EStatus CIsLightVisited::Tick() {
      m_cActuators.SetLEDs(CColor::GRAY50);
      return m_sBoard.LightVisited[m_unLight] ? EStatus::SUCCESS : EStatus::FAILURE;
   }

   EStatus CIsLightKnown::Tick() {
      m_cActuators.SetLEDs(PINK);
      return m_sBoard.LightKnown[m_unLight] ? EStatus::SUCCESS : EStatus::FAILURE;
   }

   EStatus CGoTo::Tick() {
      m_cActuators.SetLEDs(CColor::YELLOW);

      /* Read every tick: a light's estimate improves as the robot gets closer */
      CVector2 cTarget = m_fnTarget();
      Real fDistance = (cTarget - m_sBoard.Position).Length();

      if (!m_bStarted) {
         m_bStarted = true;
         m_fBestDistance = fDistance;
         m_fBestTime = m_sBoard.Time;
         /* Restarting at a leave point (on the m-line) keeps the same line */
         if (m_psMLine) {
            m_psMLine->Start = m_sBoard.Position;
            m_psMLine->Valid = true;
         }
      }

      if (fDistance < ARRIVED) {
         m_cActuators.Stop();
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      if (fDistance <= m_fBestDistance - PROGRESS) {
         m_fBestDistance = fDistance;
         m_fBestTime = m_sBoard.Time;
      }
      else if (m_sBoard.Time - m_fBestTime > TIMEOUT) {
         m_cActuators.Stop();
         m_bStarted = false;
         return EStatus::FAILURE;
      }

      m_cActuators.DriveTo(m_sBoard, cTarget, SPEED);
      return EStatus::RUNNING;
   }

   EStatus CFollowWall::Tick() {
      m_cActuators.SetLEDs(CColor::MAGENTA);

      CVector2 cTarget = m_fnTarget();
      Real fDistance = (cTarget - m_sBoard.Position).Length();

      /* At the hit point: keep the wall on the side the obstacle is on */
      if (!m_bStarted) {
         m_bStarted = true;
         Real fRight = std::min({ m_sBoard.Proximity[0], m_sBoard.Proximity[1], m_sBoard.Proximity[2] });
         Real fLeft = std::min({ m_sBoard.Proximity[5], m_sBoard.Proximity[6], m_sBoard.Proximity[7] });
         if (fRight < fLeft) m_bRight = true;
         else if (fLeft < fRight) m_bRight = false;
         else m_bRight = !m_bRight;
         m_cHitPoint = m_sBoard.Position;
         m_fHitDistance = fDistance;
         m_bLeftHitPoint = false;
         m_fStartTime = m_sBoard.Time;
         m_fPrevError = 0.0;
         if (!m_psMLine->Valid) {
            m_psMLine->Start = m_sBoard.Position;
            m_psMLine->Valid = true;
         }
      }

      if (fDistance < ARRIVED) {
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      Real fFromHit = (m_sBoard.Position - m_cHitPoint).Length();
      if (fFromHit > HIT_POINT_LEFT) m_bLeftHitPoint = true;

      /* Bug2's leave point */
      if (m_bLeftHitPoint &&
          fDistance < m_fHitDistance - LEAVE_GAIN &&
          DistanceToMLine(cTarget) < MLINE_TOLERANCE) {
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      /* Bug2's failure: back at the hit point without a leave point */
      if ((m_bLeftHitPoint && fFromHit < HIT_POINT_BACK) ||
          m_sBoard.Time - m_fStartTime > FOLLOW_TIMEOUT) {
         m_cActuators.Stop();
         m_bStarted = false;
         return EStatus::FAILURE;
      }

      m_cActuators.FollowWall(m_sBoard, m_bRight, m_fPrevError, FOLLOW_SPEED);
      return EStatus::RUNNING;
   }

   Real CFollowWall::DistanceToMLine(const CVector2& c_target) const {
      CVector2 cLine = c_target - m_psMLine->Start;
      CVector2 cToRobot = m_sBoard.Position - m_psMLine->Start;
      Real fLength = cLine.Length();
      if (fLength < 1e-6) return cToRobot.Length();
      return std::abs(cLine.GetX() * cToRobot.GetY() - cLine.GetY() * cToRobot.GetX()) / fLength;
   }

   EStatus CWalkAround::Tick() {
      m_cActuators.SetLEDs(CColor::CYAN);

      if (!m_bStarted) {
         m_bStarted = true;
         CRadians cDirection = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
         m_cGoal = m_sBoard.Position + CVector2(2.0, cDirection);
         m_fEndTime = m_sBoard.Time + m_fDuration;
      }

      if (m_sBoard.Time >= m_fEndTime) {
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      m_cActuators.DriveTo(m_sBoard, m_cGoal, SPEED);
      return EStatus::RUNNING;
   }

   EStatus CSearchLight::Tick() {
      if (m_sBoard.LightKnown[m_unLight]) {
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      m_cActuators.SetLEDs(CColor::ORANGE);

      /* A new random direction every SEARCH_LEG seconds */
      if (!m_bStarted || m_sBoard.Time >= m_fLegEnd) {
         m_bStarted = true;
         CRadians cDirection = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
         m_cGoal = m_sBoard.Position + CVector2(2.0, cDirection);
         m_fLegEnd = m_sBoard.Time + SEARCH_LEG;
      }

      m_cActuators.DriveTo(m_sBoard, m_cGoal, SPEED);
      return EStatus::RUNNING;
   }

   EStatus CVisit::Tick() {
      m_cActuators.SetLEDs(CColor::WHITE);
      m_cActuators.Stop();

      if (!m_bStarted) {
         m_bStarted = true;
         m_fEndTime = m_sBoard.Time + VISIT_TIME;
      }

      if ((m_sBoard.LightPosition[m_unLight] - m_sBoard.Position).Length() > VISIT_RADIUS) {
         m_bStarted = false;
         return EStatus::FAILURE;
      }

      if (m_sBoard.Time >= m_fEndTime) {
         m_sBoard.LightVisited[m_unLight] = true;
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      return EStatus::RUNNING;
   }

}
