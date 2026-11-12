#ifndef EX2_LOOP_FUNCTIONS_H
#define EX2_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/color.h>
#include "../sensors/battery.hpp"
#include <string>
#include <vector>

namespace argos {

   class CPiPuckEntity;

   /*
    * The Exercise 2 referee. DO NOT MODIFY.
    */
   class CEx2LoopFunctions : public CLoopFunctions {

   public:

      struct SLight {
         std::string Name;
         CColor Color;
         CVector2 Position;
         Real VisitedAt = -1.0;   // seconds; -1 = not visited yet
      };

      void Init(TConfigurationNode& t_tree) override;
      void Reset() override;
      void PreStep() override;
      void PostStep() override;
      bool IsExperimentFinished() override;
      void PostExperiment() override;

      /* For drawing (see qt_user_functions.cpp) */
      const std::vector<SLight>& GetLights() const { return m_vecLights; }
      const CVector2& GetHome() const { return m_cHome; }
      Real GetBattery() const { return m_cBattery.Level(); }
      bool IsHomeDone() const { return m_fHomeAt >= 0.0; }
      CVector2 GetRobotPosition() const;
      static constexpr Real VISIT_RADIUS = 0.15;

   private:

      Real Now() const;
      bool IsClear(const CVector2& c_point, Real f_clearance) const;
      void PlaceRobot();
      void PlaceLights();

      CPiPuckEntity* m_pcRobot = nullptr;
      std::vector<SLight> m_vecLights;   /* in the order they must be visited */
      CVector2 m_cHome;
      CVector2 m_cLastPosition;
      CBattery m_cBattery;                         /* the battery that counts */
      CBatteryEquipped* m_pcRobotBattery = nullptr; /* the controller's copy of it */
      bool m_bDead = false;
      Real m_fHomeAt = -1.0;
      size_t m_unNext = 0;           /* the next light to visit; 3 = go home */
      UInt32 m_unStillSteps = 0;     /* steps in a row standing "still" at the current target */
   };
}

#endif
