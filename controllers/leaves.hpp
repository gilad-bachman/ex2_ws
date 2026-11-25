/*
 * Exercise 2 - ID: 211397054
 *
 * The leaves: the controllers at the bottom of the tree. Each does one job,
 * one step per tick, lights the LEDs in its own color, and answers
 * RUNNING, SUCCESS ("I'm done") or FAILURE ("I can't").
 *
 * LED colors: not red, green or blue. The camera sees the "leds" medium, the
 * same one the lights are in, so those colors could be mistaken for a light.
 *
 *    GoTo         yellow
 *    FollowWall   magenta
 *    WalkAround   cyan
 *    SearchLight  orange
 *    Visit        white
 *    Charge       purple
 *    StayHome     brown
 *
 * Conditions (IsLightVisited?, IsLightKnown?, IsBatteryLow?) answer at once
 * and touch no LEDs.
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

   /* Every leaf: what it knows (the blackboard) and how it moves (the actuators) */
   class CLeaf : public CNode {

   public:

      CLeaf(SBlackboard& s_board, CActuators& c_actuators)
         : m_sBoard(s_board), m_cActuators(c_actuators) {}

      /* Halted: next time it is ticked, it starts over */
      void Halt() override { m_bStarted = false; }

   protected:

      SBlackboard& m_sBoard;
      CActuators& m_cActuators;

      /* false until the first tick, and again after it ends or is halted */
      bool m_bStarted = false;
   };

   /*
    * Bug2's m-line (Lecture 7): the straight line from where the robot started
    * heading for a target, to that target. GoTo drives along it and records
    * its start; FollowWall leaves an obstacle only where it meets it again.
    * One per GetThere subtree, shared by its GoTo and FollowWall.
    */
   struct SMLine {
      CVector2 Start;       // world frame, in m
      bool Valid = false;   // false until GoTo has started once
   };

   /*
    * GoTo: drive to a point, around obstacles (DriveTo). Bug2's "move along
    * the m-line" step: it heads straight for the target from where it starts.
    *    SUCCESS: within ARRIVED of the point
    *    FAILURE: no progress (PROGRESS closer) for TIMEOUT seconds: an obstacle (the hit point)
    * The point is read every tick, through a function, so it can come from
    * the blackboard (for example a light seen earlier) and improve on the way.
    * If given an m-line, it records where it started on it.
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
      Real m_fBestDistance = 0.0;   // the closest it has been, in m
      Real m_fBestTime = 0.0;       // when, in s
   };

   /*
    * FollowWall: Bug2's boundary following (Lecture 7, slide 6). Starts at the
    * hit point (where GoTo got stuck) and follows the obstacle's edge, the
    * wall kept on one side (the side the obstacle is seen on; if unclear, the
    * other way around than last time) for the whole traversal.
    *    SUCCESS: on the m-line again, strictly closer to the target than the
    *             hit point (by LEAVE_GAIN): the leave point, GoTo takes over
    *    FAILURE: back at the hit point with no leave point (Bug2's failure
    *             detection), or FOLLOW_TIMEOUT seconds (a safety net)
    */
   class CFollowWall : public CLeaf {

   public:

      CFollowWall(SBlackboard& s_board, CActuators& c_actuators,
                  std::function<CVector2()> fn_target,
                  std::shared_ptr<SMLine> ps_mline)
         : CLeaf(s_board, c_actuators), m_fnTarget(std::move(fn_target)), m_psMLine(std::move(ps_mline)) {}

      EStatus Tick() override;

   private:

      /* How far the robot is from the m-line (start to c_target), in m */
      Real DistanceToMLine(const CVector2& c_target) const;

      std::function<CVector2()> m_fnTarget;
      std::shared_ptr<SMLine> m_psMLine;
      bool m_bRight = false;          // the wall on the right?
      CVector2 m_cHitPoint;           // where the following started
      Real m_fHitDistance = 0.0;      // from the hit point to the target, in m
      bool m_bLeftHitPoint = false;   // has it moved away from the hit point yet?
      Real m_fStartTime = 0.0;        // in s
      Real m_fPrevError = 0.0;        // the wall PD's memory
   };

   /*
    * WalkAround: drive in a random direction, around obstacles, for a while.
    * A way out when GoTo is stuck.
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
      CVector2 m_cGoal;      // a point 2 m away in the random direction
      Real m_fEndTime = 0.0;
   };

   /* A condition about one light: a question answered at once, nothing to halt */
   class CLightCondition : public CNode {

   public:

      CLightCondition(const SBlackboard& s_board, size_t un_light)
         : m_sBoard(s_board), m_unLight(un_light) {}

      void Halt() override {}

   protected:

      const SBlackboard& m_sBoard;
      size_t m_unLight;   // index in Order
   };

   /* IsLightVisited?: SUCCESS if light un_light was already visited */
   class CIsLightVisited : public CLightCondition {
   public:
      using CLightCondition::CLightCondition;
      EStatus Tick() override;
   };

   /* IsLightKnown?: SUCCESS if the robot knows where light un_light is */
   class CIsLightKnown : public CLightCondition {
   public:
      using CLightCondition::CLightCondition;
      EStatus Tick() override;
   };

   /*
    * IsBatteryLow?: SUCCESS if the battery is too low to keep going, given how
    * far the charger is (threshold in leaves.cpp: twice what the straight trip
    * home costs, plus a reserve).
    */
   class CIsBatteryLow : public CNode {

   public:

      explicit CIsBatteryLow(const SBlackboard& s_board) : m_sBoard(s_board) {}

      EStatus Tick() override;
      void Halt() override {}

   private:

      const SBlackboard& m_sBoard;
   };

   /*
    * Charge: stand still on the charger until the battery is full.
    *    SUCCESS: full
    *    FAILURE: not on the charger
    */
   class CCharge : public CLeaf {

   public:

      using CLeaf::CLeaf;

      EStatus Tick() override;
   };

   /*
    * StayHome: the end of the mission. Stand still on the charger long enough
    * for the referee to count coming home.
    *    SUCCESS: still for VISIT_TIME on the charger
    *    FAILURE: not on the charger
    */
   class CStayHome : public CLeaf {

   public:

      using CLeaf::CLeaf;

      EStatus Tick() override;

   private:

      Real m_fEndTime = 0.0;
   };

   /*
    * SearchLight: a random walk, around obstacles, until light un_light is known.
    *    SUCCESS: the light is known (seen now, or before)
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
      CVector2 m_cGoal;      // where the current leg heads
      Real m_fLegEnd = 0.0;  // when to pick a new direction
   };

   /*
    * Visit: stand still at light un_light long enough for the referee to count
    * it, then mark it visited on the blackboard.
    *    SUCCESS: still for VISIT_TIME, near the light
    *    FAILURE: drifted away from the light
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
