/*
 * Exercise 2 - ID: 211397054
 */
#include "leaves.hpp"
#include "battery.hpp"

#include <algorithm>
#include <cmath>

namespace argos {

   namespace {

      const Real SPEED = CCI_PiPuckDifferentialDriveActuator::MAX_WHEEL_VELOCITY;   // m/s: full speed (0.154)
      const Real ARRIVED = 0.05;          // m: this close to the point is there
      const Real PROGRESS = 0.05;         // m: getting this much closer counts as progress
      const Real TIMEOUT = 5.0;           // s without progress: stuck (FollowWall takes over)

      /* Bug2 (Lecture 7, slide 6) */
      const Real FOLLOW_SPEED = 0.1;      // m/s: slower than SPEED, to hug the wall
      const Real LEAVE_GAIN = 0.05;       // m: "strictly closer" than the hit point, by at least this much
      const Real MLINE_TOLERANCE = 0.04;  // m: this close to the m-line counts as on it
      const Real HIT_POINT_LEFT = 0.15;   // m: this far from the hit point: it has left it
      const Real HIT_POINT_BACK = 0.08;   // m: this close again, after leaving it: back at the hit point
      const Real FOLLOW_TIMEOUT = 20.0;   // s: a safety net, the boundary is too long (16% of the battery)

      const Real SEARCH_LEG = 5.0;        // s: Search drives this long in one direction
      const Real VISIT_TIME = 2.5;        // s: the referee needs 2 s still; a little margin
      const Real VISIT_RADIUS = 0.12;     // m: drifted farther than this from the light: failed (referee: 0.15)

      /*
       * The battery threshold, from the README's battery rule:
       *    drain 0.8 %/s (CBattery::DRAIN_PER_SECOND), driving at SPEED (0.154 m/s)
       *    -> the straight trip home costs DRAIN / SPEED = about 5.2 % per meter.
       * The real way home is longer (obstacles, walking around), so allow
       * HOME_SAFETY times that, plus a fixed reserve.
       */
      const Real HOME_SAFETY = 2.0;
      const Real BATTERY_RESERVE = 5.0;   // %
      const Real CHARGER_RADIUS = 0.12;   // m: on the charger (referee: CBattery::CHARGER_RADIUS = 0.15)
      const Real FULL = 99.0;             // %: full enough
   }

   /* ---------------- Battery ---------------- */

   EStatus CIsBatteryLow::Tick() {
      Real fDistance = (m_sBoard.Home - m_sBoard.Position).Length();
      Real fTripCost = CBattery::DRAIN_PER_SECOND * fDistance / SPEED;   // % for the straight trip home
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

   /* ---------------- StayHome ---------------- */

   EStatus CStayHome::Tick() {
      m_cActuators.SetLEDs(CColor::BROWN);
      m_cActuators.Stop();

      if (!m_bStarted) {
         m_bStarted = true;
         m_fEndTime = m_sBoard.Time + VISIT_TIME;   // same 2.5 s as a light visit
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

   /* ---------------- Conditions ---------------- */

   EStatus CIsLightVisited::Tick() {
      return m_sBoard.LightVisited[m_unLight] ? EStatus::SUCCESS : EStatus::FAILURE;
   }

   EStatus CIsLightKnown::Tick() {
      return m_sBoard.LightKnown[m_unLight] ? EStatus::SUCCESS : EStatus::FAILURE;
   }

   /* ---------------- GoTo ---------------- */

   EStatus CGoTo::Tick() {
      m_cActuators.SetLEDs(CColor::YELLOW);

      /* Read the target every tick: a light's position gets better as the robot sees it again */
      CVector2 cTarget = m_fnTarget();
      Real fDistance = (cTarget - m_sBoard.Position).Length();

      if (!m_bStarted) {
         m_bStarted = true;
         m_fBestDistance = fDistance;
         m_fBestTime = m_sBoard.Time;
         /* Bug2: the m-line starts here. Starting again at a leave point (on
            the m-line) keeps the same line, since it heads for the same target */
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

   /* ---------------- FollowWall ---------------- */

   EStatus CFollowWall::Tick() {
      m_cActuators.SetLEDs(CColor::MAGENTA);

      CVector2 cTarget = m_fnTarget();
      Real fDistance = (cTarget - m_sBoard.Position).Length();

      /* The hit point: where the boundary following starts */
      if (!m_bStarted) {
         m_bStarted = true;
         /* The wall on the side the obstacle is on, kept for the whole
            traversal; if both sides look the same, the other way than last time */
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
         /* No m-line yet (GoTo never ran): it starts at the hit point */
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

      /* Bug2's leave point: on the m-line again, strictly closer than the hit point */
      if (m_bLeftHitPoint &&
          fDistance < m_fHitDistance - LEAVE_GAIN &&
          DistanceToMLine(cTarget) < MLINE_TOLERANCE) {
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      /* Bug2's failure: all the way around, back at the hit point, no leave point */
      if (m_bLeftHitPoint && fFromHit < HIT_POINT_BACK) {
         m_cActuators.Stop();
         m_bStarted = false;
         return EStatus::FAILURE;
      }

      /* A safety net: the boundary is too long for the battery */
      if (m_sBoard.Time - m_fStartTime > FOLLOW_TIMEOUT) {
         m_cActuators.Stop();
         m_bStarted = false;
         return EStatus::FAILURE;
      }

      m_cActuators.FollowWall(m_sBoard, m_bRight, m_fPrevError, FOLLOW_SPEED);
      return EStatus::RUNNING;
   }

   Real CFollowWall::DistanceToMLine(const CVector2& c_target) const {
      /* Distance from a point to the line through Start and the target: |cross| / length */
      CVector2 cLine = c_target - m_psMLine->Start;
      CVector2 cToRobot = m_sBoard.Position - m_psMLine->Start;
      Real fLength = cLine.Length();
      if (fLength < 1e-6) return cToRobot.Length();
      return std::abs(cLine.GetX() * cToRobot.GetY() - cLine.GetY() * cToRobot.GetX()) / fLength;
   }

   /* ---------------- WalkAround ---------------- */

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

   /* ---------------- Search ---------------- */

   EStatus CSearchLight::Tick() {
      /* Done as soon as the light is known (Controller2 records every sighting) */
      if (m_sBoard.LightKnown[m_unLight]) {
         m_bStarted = false;
         return EStatus::SUCCESS;
      }

      m_cActuators.SetLEDs(CColor::ORANGE);

      /* A random walk: a new random direction every SEARCH_LEG seconds */
      if (!m_bStarted || m_sBoard.Time >= m_fLegEnd) {
         m_bStarted = true;
         CRadians cDirection = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
         m_cGoal = m_sBoard.Position + CVector2(2.0, cDirection);
         m_fLegEnd = m_sBoard.Time + SEARCH_LEG;
      }

      m_cActuators.DriveTo(m_sBoard, m_cGoal, SPEED);
      return EStatus::RUNNING;
   }

   /* ---------------- Visit ---------------- */

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
