/*
 * Exercise 2 - ID: 211397054
 *
 * The leaves: each does one job per tick, lights its own LED color, and
 * answers RUNNING, SUCCESS or FAILURE. No red, green or blue: the camera
 * would take them for a light.
 *
 *    GoTo yellow, FollowWall magenta, WalkAround cyan, SearchLight orange,
 *    Visit white, Charge purple, StayHome brown,
 *    IsLightVisited? gray, IsLightKnown? pink, IsBatteryLow? light gray
 */
#ifndef LEAVES_HPP
#define LEAVES_HPP

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <functional>
#include <memory>

#include "behavior_tree.hpp"
#include "blackboard.hpp"
#include "actuators.hpp"

namespace argos {

   class CLeaf : public CNode {

   public:

      CLeaf(SBlackboard& s_board, CActuators& c_actuators)
         : m_sBoard(s_board), m_cActuators(c_actuators) {}

      void Halt() override { m_bStarted = false; }

   protected:

      SBlackboard& m_sBoard;
      CActuators& m_cActuators;
      bool m_bStarted = false;
   };

   /* Bug2's m-line (Lecture 7): shared by one GetThere subtree's GoTo and FollowWall */
   struct SMLine {
      CVector2 Start;
      bool Valid = false;
   };

   /*
    * GoTo: drive to a point (along the m-line, if given one).
    *    SUCCESS: arrived. FAILURE: no progress for a while (Bug2's hit point).
    */
   class CGoTo : public CLeaf {

   public:

      CGoTo(SBlackboard& s_board, CActuators& c_actuators,
            std::function<CVector2()> fn_target,
            std::shared_ptr<SMLine> ps_mline = nullptr)
         : CLeaf(s_board, c_actuators), m_fnTarget(std::move(fn_target)), m_psMLine(std::move(ps_mline)) {}

      EStatus Tick() override;

   private:

      std::function<CVector2()> m_fnTarget;
      std::shared_ptr<SMLine> m_psMLine;
      Real m_fBestDistance = 0.0;
      Real m_fBestTime = 0.0;
   };

   /*
    * FollowWall: Bug2's boundary following (Lecture 7, slide 6).
    *    SUCCESS: on the m-line again, closer to the target than the hit point.
    *    FAILURE: back at the hit point, or a timeout.
    */
   class CFollowWall : public CLeaf {

   public:

      CFollowWall(SBlackboard& s_board, CActuators& c_actuators,
                  std::function<CVector2()> fn_target,
                  std::shared_ptr<SMLine> ps_mline)
         : CLeaf(s_board, c_actuators), m_fnTarget(std::move(fn_target)), m_psMLine(std::move(ps_mline)) {}

      EStatus Tick() override;

   private:

      Real DistanceToMLine(const CVector2& c_target) const;

      std::function<CVector2()> m_fnTarget;
      std::shared_ptr<SMLine> m_psMLine;
      bool m_bRight = false;
      CVector2 m_cHitPoint;
      Real m_fHitDistance = 0.0;
      bool m_bLeftHitPoint = false;
      Real m_fStartTime = 0.0;
      Real m_fPrevError = 0.0;
   };

   /* WalkAround: drive in a random direction for f_duration seconds, then SUCCESS */
   class CWalkAround : public CLeaf {

   public:

      CWalkAround(SBlackboard& s_board, CActuators& c_actuators,
                  CRandom::CRNG* pc_rng, Real f_duration = 6.0)
         : CLeaf(s_board, c_actuators), m_pcRNG(pc_rng), m_fDuration(f_duration) {}

      EStatus Tick() override;

   private:

      CRandom::CRNG* m_pcRNG;
      Real m_fDuration;
      CVector2 m_cGoal;
      Real m_fEndTime = 0.0;
   };

   /* A condition about one light (its index in the order): answers at once */
   class CLightCondition : public CLeaf {

   public:

      CLightCondition(SBlackboard& s_board, CActuators& c_actuators, size_t un_light)
         : CLeaf(s_board, c_actuators), m_unLight(un_light) {}

   protected:

      size_t m_unLight;
   };

   class CIsLightVisited : public CLightCondition {
   public:
      using CLightCondition::CLightCondition;
      EStatus Tick() override;
   };

   class CIsLightKnown : public CLightCondition {
   public:
      using CLightCondition::CLightCondition;
      EStatus Tick() override;
   };

   /* IsBatteryLow?: SUCCESS if the battery is too low for the way home (threshold in leaves.cpp) */
   class CIsBatteryLow : public CLeaf {
   public:
      using CLeaf::CLeaf;
      EStatus Tick() override;
   };

   /* Charge: stand on the charger until full. FAILURE if not on it. */
   class CCharge : public CLeaf {
   public:
      using CLeaf::CLeaf;
      EStatus Tick() override;
   };

   /* StayHome: stand still on the charger long enough to count as home */
   class CStayHome : public CLeaf {

   public:

      using CLeaf::CLeaf;

      EStatus Tick() override;

   private:

      Real m_fEndTime = 0.0;
   };

   /* SearchLight: random walk until the light is known. Never fails. */
   class CSearchLight : public CLeaf {

   public:

      CSearchLight(SBlackboard& s_board, CActuators& c_actuators,
                   CRandom::CRNG* pc_rng, size_t un_light)
         : CLeaf(s_board, c_actuators), m_pcRNG(pc_rng), m_unLight(un_light) {}

      EStatus Tick() override;

   private:

      CRandom::CRNG* m_pcRNG;
      size_t m_unLight;
      CVector2 m_cGoal;
      Real m_fLegEnd = 0.0;
   };

   /* Visit: stand still at the light long enough to count, then mark it visited */
   class CVisit : public CLeaf {

   public:

      CVisit(SBlackboard& s_board, CActuators& c_actuators, size_t un_light)
         : CLeaf(s_board, c_actuators), m_unLight(un_light) {}

      EStatus Tick() override;

   private:

      size_t m_unLight;
      Real m_fEndTime = 0.0;
   };

}

#endif
