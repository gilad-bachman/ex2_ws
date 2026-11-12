#include "loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <algorithm>
#include <cmath>

/* The order of the lights, the same parameter the controller reads */
#include "../sensors/light_order.hpp"

namespace argos {

   /* The robot has this much simulated time, in seconds */
   static const Real TIME_LIMIT = 600.0;
   /* The arena: 4 m x 4 m, from (-2,-2) to (2,2) */
   static const Real ARENA_HALF = 2.0;
   /* The random start keeps at least this far from walls and obstacles, in meters */
   static const Real START_CLEARANCE = 0.2;
   /* The lights keep at least this far from walls and obstacles, in meters */
   static const Real LIGHT_CLEARANCE = 0.25;
   /* ...and at least this far from each other and from the charger, in meters */
   static const Real LIGHT_SPACING = 1.0;
   /* ...and at most this far from the charger, in meters, so one charge is enough to get there and back */
   static const Real LIGHT_MAX_FROM_HOME = 3.5;
   /* Standing still: slower than this, in m/s */
   static const Real STILL_SPEED = 0.01;
   /* Standing still for this long counts as a visit, in seconds */
   static const Real VISIT_TIME = 2.0;
   /* The time of one step, in seconds */
   static const Real STEP = 0.1;

   /* Distance from a point to a box seen from above. All boxes are axis-aligned. */
   static Real DistanceToBox(const CVector2& c_point, CBoxEntity& c_box) {
      const CVector3& cCenter = c_box.GetEmbodiedEntity().GetOriginAnchor().Position;
      const CVector3& cSize = c_box.GetSize();
      Real fDX = std::max(Abs(c_point.GetX() - cCenter.GetX()) - cSize.GetX() / 2, 0.0);
      Real fDY = std::max(Abs(c_point.GetY() - cCenter.GetY()) - cSize.GetY() / 2, 0.0);
      return std::hypot(fDX, fDY);
   }

   static CVector2 Flat(const CVector3& c) { return CVector2(c.GetX(), c.GetY()); }

   /****************************************/
   /****************************************/

   CVector2 CEx2LoopFunctions::GetRobotPosition() const {
      CVector3 cPosition = m_pcRobot->GetEmbodiedEntity().GetOriginAnchor().Position;
      return Flat(cPosition);
   }

   Real CEx2LoopFunctions::Now() const {
      return GetSpace().GetSimulationClock() * CPhysicsEngine::GetSimulationClockTick();
   }

   /* Is the point inside the arena, and at least f_clearance from every box? */
   bool CEx2LoopFunctions::IsClear(const CVector2& c_point, Real f_clearance) const {
      if (Abs(c_point.GetX()) > ARENA_HALF - 0.05 - f_clearance) return false;
      if (Abs(c_point.GetY()) > ARENA_HALF - 0.05 - f_clearance) return false;
      CSpace::TMapPerType& tBoxes = const_cast<CEx2LoopFunctions*>(this)->GetSpace().GetEntitiesByType("box");
      for (auto& tPair : tBoxes) {
         if (DistanceToBox(c_point, *any_cast<CBoxEntity*>(tPair.second)) < f_clearance) return false;
      }
      return true;
   }

   /****************************************/
   /****************************************/

   void CEx2LoopFunctions::Init(TConfigurationNode& t_tree) {
      CSpace::TMapPerType& tRobots = GetSpace().GetEntitiesByType("pipuck");
      m_pcRobot = any_cast<CPiPuckEntity*>(tRobots.begin()->second);
      /* The controller's battery, which the referee writes every step */
      m_pcRobotBattery = dynamic_cast<CBatteryEquipped*>(&m_pcRobot->GetControllableEntity().GetController());
      if (m_pcRobotBattery == nullptr)
         THROW_ARGOSEXCEPTION("Exercise 2 referee: the controller must inherit from CBatteryEquipped (sensors/battery.hpp)");
      /* The order: <params order="..."/> of the controller in the .argos file */
      TConfigurationNode& tControllers = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "controllers");
      TConfigurationNodeIterator itController;
      itController = itController.begin(&tControllers);
      if (itController == itController.end()) THROW_ARGOSEXCEPTION("Exercise 2 referee: no controller in the .argos file");
      for (const SLightInfo& s : ReadLightOrder(GetNode(*itController, "params")))
         m_vecLights.push_back({ s.Name, s.Color, CVector2() });
      Reset();
   }

   /****************************************/
   /****************************************/

   void CEx2LoopFunctions::PlaceRobot() {
      CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
      CRange<Real> cRange(-ARENA_HALF, ARENA_HALF);
      for (UInt32 unTry = 0; unTry < 10000; ++unTry) {
         CVector2 cPoint(pcRNG->Uniform(cRange), pcRNG->Uniform(cRange));
         if (!IsClear(cPoint, START_CLEARANCE)) continue;
         CRadians cHeading = pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
         if (MoveEntity(m_pcRobot->GetEmbodiedEntity(), CVector3(cPoint.GetX(), cPoint.GetY(), 0),
                        CQuaternion(cHeading, CVector3::Z))) {
            m_cHome = cPoint;
            return;
         }
      }
      THROW_ARGOSEXCEPTION("Exercise 2 referee: could not find a free start position");
   }

   /****************************************/
   /****************************************/

   void CEx2LoopFunctions::PlaceLights() {
      CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
      CRange<Real> cRange(-ARENA_HALF, ARENA_HALF);
      std::vector<CVector2> vecTaken = { m_cHome };
      /* Placed always in the same color order, so the same seed puts each color in the same place, whatever the order */
      for (const std::string& strName : { "red", "green", "blue" }) {
         SLight& sLight = *std::find_if(m_vecLights.begin(), m_vecLights.end(), [&](const SLight& s) { return s.Name == strName; });
         bool bPlaced = false;
         for (UInt32 unTry = 0; unTry < 10000 && !bPlaced; ++unTry) {
            CVector2 cPoint(pcRNG->Uniform(cRange), pcRNG->Uniform(cRange));
            if (!IsClear(cPoint, LIGHT_CLEARANCE)) continue;
            if ((cPoint - m_cHome).Length() > LIGHT_MAX_FROM_HOME) continue;
            bool bSpaced = true;
            for (const CVector2& c : vecTaken) if ((c - cPoint).Length() < LIGHT_SPACING) bSpaced = false;
            if (!bSpaced) continue;
            sLight.Position = cPoint;
            vecTaken.push_back(cPoint);
            bPlaced = true;
         }
         if (!bPlaced) THROW_ARGOSEXCEPTION("Exercise 2 referee: could not place the " << sLight.Name << " light");
         /* Move the light (it is in the .argos file) */
         auto& cLight = dynamic_cast<CLightEntity&>(GetSpace().GetEntity("light_" + sLight.Name));
         cLight.SetPosition(CVector3(sLight.Position.GetX(), sLight.Position.GetY(), cLight.GetPosition().GetZ()));
      }
   }

   /****************************************/
   /****************************************/

   void CEx2LoopFunctions::Reset() {
      PlaceRobot();
      PlaceLights();
      for (SLight& s : m_vecLights) s.VisitedAt = -1.0;
      m_cLastPosition = m_cHome;
      m_cBattery.Reset(m_cHome);
      m_pcRobotBattery->m_cBattery = m_cBattery;
      m_bDead = false;
      m_fHomeAt = -1.0;
      m_unNext = 0;
      m_unStillSteps = 0;
      LOG << "[EX2] order:";
      for (const SLight& s : m_vecLights) LOG << " " << s.Name;
      LOG << std::endl;
      LOG << "[EX2] charger at (" << m_cHome.GetX() << ", " << m_cHome.GetY() << ")" << std::endl;
      for (const SLight& s : m_vecLights)
         LOG << "[EX2] " << s.Name << " light at (" << s.Position.GetX() << ", " << s.Position.GetY() << ")" << std::endl;
   }

   /****************************************/
   /****************************************/

   void CEx2LoopFunctions::PreStep() {
      /* The battery, at the position the robot senses this step, written into the controller before its ControlStep() */
      m_cBattery.Step(GetRobotPosition(), STEP);
      m_pcRobotBattery->m_cBattery = m_cBattery;
      if (m_cBattery.Level() <= 0.0 && !m_bDead) {
         m_bDead = true;
         LOG << "[EX2] " << Now() << " s: the battery is empty. The robot is dead" << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void CEx2LoopFunctions::PostStep() {
      CVector3 cPosition = m_pcRobot->GetEmbodiedEntity().GetOriginAnchor().Position;
      CVector2 cHere = Flat(cPosition);
      /* Visits: still, close to the current target, for VISIT_TIME */
      bool bStill = (cHere - m_cLastPosition).Length() / STEP < STILL_SPEED;
      m_cLastPosition = cHere;
      if (m_fHomeAt >= 0.0) return;
      bool bClose = (m_unNext < m_vecLights.size())
         ? (cHere - m_vecLights[m_unNext].Position).Length() < VISIT_RADIUS
         : (cHere - m_cHome).Length() < VISIT_RADIUS;
      m_unStillSteps = (bStill && bClose) ? m_unStillSteps + 1 : 0;
      if (m_unStillSteps * STEP >= VISIT_TIME - 1e-6) {
         m_unStillSteps = 0;
         if (m_unNext < m_vecLights.size()) {
            m_vecLights[m_unNext].VisitedAt = Now();
            LOG << "[EX2] " << Now() << " s: visited the " << m_vecLights[m_unNext].Name << " light" << std::endl;
            ++m_unNext;
         } else {
            m_fHomeAt = Now();
            LOG << "[EX2] " << Now() << " s: home. Mission complete" << std::endl;
         }
      }
   }

   /****************************************/
   /****************************************/

   bool CEx2LoopFunctions::IsExperimentFinished() {
      return m_bDead || m_fHomeAt >= 0.0 || Now() >= TIME_LIMIT;
   }

   /****************************************/
   /****************************************/

   void CEx2LoopFunctions::PostExperiment() {
      LOG << "[EX2] ---------------- result ----------------" << std::endl;
      LOG << "[EX2] time:            " << Now() << " s" << std::endl;
      for (const SLight& s : m_vecLights) {
         LOG << "[EX2] " << s.Name << " light:   ";
         if (s.VisitedAt >= 0.0) LOG << "visited at " << s.VisitedAt << " s" << std::endl;
         else LOG << "not visited" << std::endl;
      }
      LOG << "[EX2] home:          " << (m_fHomeAt >= 0.0 ? "yes" : "no") << std::endl;
      LOG << "[EX2] battery:       " << m_cBattery.Level() << " %" << (m_bDead ? " (dead)" : "") << std::endl;
      LOG.Flush();
   }

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CEx2LoopFunctions, "ex2_loop_functions");

}
