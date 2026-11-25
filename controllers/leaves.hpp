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
    * GoTo (yellow): drives straight to a target point, sliding around small
    * obstacles on the way. In Bug2 this is "move along the m-line".
    *    SUCCESS: within 5 cm of the target
    *    FAILURE: no progress for 5 s, blocked by an obstacle (Bug2's hit point)
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
    * FollowWall (magenta): gets around an obstacle that blocked GoTo by
    * following its edge, keeping it on one side (Bug2, Lecture 7, slide 6).
    *    SUCCESS: back on the m-line, closer to the target than where it
    *             started: GoTo can take over again
    *    FAILURE: went all the way around (back at the start), or 20 s
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

   /*
    * WalkAround (cyan): drives in a random direction for a few seconds, to
    * get the robot out of a spot where FollowWall could not help.
    *    SUCCESS: after f_duration seconds
    *    FAILURE: never
    */
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

   /*
    * IsLightVisited? (gray): has this light already been visited?
    *    SUCCESS: yes, so the robot can skip it (for example after a recharge)
    *    FAILURE: no
    */
   class CIsLightVisited : public CLightCondition {
   public:
      using CLightCondition::CLightCondition;
      EStatus Tick() override;
   };

   /*
    * IsLightKnown? (pink): does the robot know where this light is, because the
    * camera has seen it at some point?
    *    SUCCESS: yes, so no need to search
    *    FAILURE: no
    */
   class CIsLightKnown : public CLightCondition {
   public:
      using CLightCondition::CLightCondition;
      EStatus Tick() override;
   };

   /*
    * IsBatteryLow? (light gray): is it time to go home and charge? The
    * threshold grows with the distance to the charger (see leaves.cpp).
    *    SUCCESS: yes, the battery is below the threshold
    *    FAILURE: no
    */
   class CIsBatteryLow : public CLeaf {
   public:
      using CLeaf::CLeaf;
      EStatus Tick() override;
   };

   /*
    * Charge (purple): stands still on the charger while the battery fills up.
    *    SUCCESS: the battery is full
    *    FAILURE: the robot is not on the charger
    */
   class CCharge : public CLeaf {
   public:
      using CLeaf::CLeaf;
      EStatus Tick() override;
   };

   /*
    * StayHome (brown): the last step of the mission. Stands still on the
    * charger for 2.5 s, so the referee counts the robot as home.
    *    SUCCESS: 2.5 s on the charger
    *    FAILURE: the robot is not on the charger
    */
   class CStayHome : public CLeaf {

   public:

      using CLeaf::CLeaf;

      EStatus Tick() override;

   private:

      Real m_fEndTime = 0.0;
   };

   /*
    * SearchLight (orange): looks for a light by driving in a new random
    * direction every 5 s, until the camera has seen it.
    *    SUCCESS: the light's position is known
    *    FAILURE: never
    */
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

   /*
    * Visit (white): stands still at a light for 2.5 s, so the referee counts
    * the visit, then marks the light as visited on the blackboard.
    *    SUCCESS: 2.5 s at the light
    *    FAILURE: the robot drifted away from the light
    */
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
