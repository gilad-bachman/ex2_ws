/*
 * Exercise 2 - ID: 211397054
 *
 * The plan: builds the mission's behavior tree out of the engine's control
 * nodes (behavior_tree.hpp) and the leaves (leaves.hpp).
 */
#ifndef MISSION_HPP
#define MISSION_HPP

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <functional>
#include <memory>

#include "behavior_tree.hpp"
#include "blackboard.hpp"
#include "actuators.hpp"
#include "light_order.hpp"

namespace argos {

   /* What every part of the tree needs */
   struct SMissionContext {
      SBlackboard& Board;
      CActuators& Actuators;
      CRandom::CRNG* RNG;
   };

   /*
    * Get to a point, and don't give up:
    *
    *    RetryUntilSuccessful
    *       Fallback
    *          GoTo(target)                along the m-line (Bug2)
    *          ForceFailure                stuck (the hit point): get unstuck, then retry GoTo
    *             Fallback
    *                FollowWall(target)    Bug2: along the boundary, leave on the m-line closer than the hit point
    *                WalkAround            Bug2 found no leave point: a random walk
    *
    * GoTo and FollowWall share the subtree's m-line (SMLine in leaves.hpp).
    * The Bug algorithms are from Lectures 6 and 7.
    */
   std::unique_ptr<CNode> MakeGetThere(const SMissionContext& s_ctx,
                                       std::function<CVector2()> fn_target);

   /*
    * Visit one light: un_index is its place in the order (0, 1, 2), s_light
    * which light it is. The same subtree serves every light.
    *
    *    Fallback
    *       IsLightVisited?(n)                 already done: skip it
    *       Sequence
    *          Fallback                         find it
    *             IsLightKnown?(n)
    *             SearchLight(n)
    *          GetThere(light n)                see MakeGetThere
    *          Visit(n)                          stand still 2.5 s, mark visited
    */
   std::unique_ptr<CNode> MakeVisitLight(const SMissionContext& s_ctx,
                                         size_t un_index,
                                         const SLightInfo& s_light);

   /*
    * Recharge, when the battery is low:
    *
    *    Sequence
    *       IsBatteryLow?
    *       GetThere(home)
    *       Charge                            until full
    *
    * Put it as the first child of a ReactiveFallback, with the mission as the
    * second: checked every tick, and when it takes over the mission is halted.
    * Once IsBatteryLow? has said yes, this Sequence keeps going (it does not
    * ask again) until the battery is full.
    */
   std::unique_ptr<CNode> MakeRecharge(const SMissionContext& s_ctx);

   /*
    * Come home, after the last light:
    *
    *    Sequence
    *       GetThere(home)                    see MakeGetThere
    *       StayHome                          stand still 2.5 s on the charger
    */
   std::unique_ptr<CNode> MakeComeHome(const SMissionContext& s_ctx);

}

#endif
