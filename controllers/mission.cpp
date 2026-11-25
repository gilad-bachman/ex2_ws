/*
 * Exercise 2 - ID: 211397054
 */
#include "mission.hpp"
#include "leaves.hpp"

namespace argos {

   std::unique_ptr<CNode> MakeGetThere(const SMissionContext& s_ctx,
                                       std::function<CVector2()> fn_target) {
      /* Bug2's m-line, shared by this subtree's GoTo (records it) and FollowWall (leaves on it) */
      auto psMLine = std::make_shared<SMLine>();

      /* Stuck: Bug2 around the obstacle; if even that fails, a random walk */
      auto pcUnstick = std::make_unique<CFallback>();
      pcUnstick->Add(std::make_unique<CFollowWall>(s_ctx.Board, s_ctx.Actuators, fn_target, psMLine));
      pcUnstick->Add(std::make_unique<CWalkAround>(s_ctx.Board, s_ctx.Actuators, s_ctx.RNG));

      /* GoTo along the m-line; if stuck, unstick, then (ForceFailure + Retry) GoTo again */
      auto pcTry = std::make_unique<CFallback>();
      pcTry->Add(std::make_unique<CGoTo>(s_ctx.Board, s_ctx.Actuators, std::move(fn_target), psMLine));
      pcTry->Add(std::make_unique<CForceFailure>(std::move(pcUnstick)));
      return std::make_unique<CRetryUntilSuccessful>(std::move(pcTry));
   }

   std::unique_ptr<CNode> MakeVisitLight(const SMissionContext& s_ctx,
                                         size_t un_index,
                                         const SLightInfo& s_light) {
      /* s_light names the light; the blackboard keeps it by its index in the order */
      (void)s_light;
      SBlackboard& sBoard = s_ctx.Board;

      /* Find it: already known, else search */
      auto pcFind = std::make_unique<CFallback>();
      pcFind->Add(std::make_unique<CIsLightKnown>(sBoard, un_index));
      pcFind->Add(std::make_unique<CSearchLight>(sBoard, s_ctx.Actuators, s_ctx.RNG, un_index));

      /* Find, get there, visit */
      auto pcDo = std::make_unique<CSequence>();
      pcDo->Add(std::move(pcFind));
      pcDo->Add(MakeGetThere(s_ctx, [&sBoard, un_index]() { return sBoard.LightPosition[un_index]; }));
      pcDo->Add(std::make_unique<CVisit>(sBoard, s_ctx.Actuators, un_index));

      /* Skip it if already visited (after a recharge, for example) */
      auto pcRoot = std::make_unique<CFallback>();
      pcRoot->Add(std::make_unique<CIsLightVisited>(sBoard, un_index));
      pcRoot->Add(std::move(pcDo));
      return pcRoot;
   }

   std::unique_ptr<CNode> MakeRecharge(const SMissionContext& s_ctx) {
      SBlackboard& sBoard = s_ctx.Board;
      auto pcRecharge = std::make_unique<CSequence>();
      pcRecharge->Add(std::make_unique<CIsBatteryLow>(sBoard));
      pcRecharge->Add(MakeGetThere(s_ctx, [&sBoard]() { return sBoard.Home; }));
      pcRecharge->Add(std::make_unique<CCharge>(sBoard, s_ctx.Actuators));
      return pcRecharge;
   }

   std::unique_ptr<CNode> MakeComeHome(const SMissionContext& s_ctx) {
      SBlackboard& sBoard = s_ctx.Board;
      auto pcComeHome = std::make_unique<CSequence>();
      pcComeHome->Add(MakeGetThere(s_ctx, [&sBoard]() { return sBoard.Home; }));
      pcComeHome->Add(std::make_unique<CStayHome>(sBoard, s_ctx.Actuators));
      return pcComeHome;
   }

}
