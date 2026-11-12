#include "qt_user_functions.hpp"
#include "loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <cstdio>

namespace argos {

   CEx2QTUserFunctions::CEx2QTUserFunctions() :
      m_cLoopFunctions(dynamic_cast<CEx2LoopFunctions&>(
         CSimulator::GetInstance().GetLoopFunctions())) {}

   /****************************************/
   /****************************************/

   void CEx2QTUserFunctions::DrawInWorld() {
      /* Around each light: a thin pale ring where a visit counts (15 cm); a thick ring once visited */
      for (const auto& sLight : m_cLoopFunctions.GetLights()) {
         CVector3 cCenter(sLight.Position.GetX(), sLight.Position.GetY(), 0.007);
         if (sLight.VisitedAt >= 0.0) {
            for (Real fR = CEx2LoopFunctions::VISIT_RADIUS; fR < CEx2LoopFunctions::VISIT_RADIUS + 0.04; fR += 0.005)
               DrawCircle(cCenter, CQuaternion(), fR, sLight.Color, false);
         } else {
            const CColor& c = sLight.Color;
            DrawCircle(cCenter, CQuaternion(), CEx2LoopFunctions::VISIT_RADIUS,
                       CColor(c.GetRed() / 2 + 127, c.GetGreen() / 2 + 127, c.GetBlue() / 2 + 127), false);
         }
      }
      /* The charger: a yellow ring; thick when the robot is home at the end */
      const CVector2& cHome = m_cLoopFunctions.GetHome();
      for (Real fR = CEx2LoopFunctions::VISIT_RADIUS; fR < CEx2LoopFunctions::VISIT_RADIUS + (m_cLoopFunctions.IsHomeDone() ? 0.04 : 0.001); fR += 0.005)
         DrawCircle(CVector3(cHome.GetX(), cHome.GetY(), 0.007), CQuaternion(), fR, CColor(200, 160, 0), false);

      CVector2 cRobot2d = m_cLoopFunctions.GetRobotPosition();
      CVector3 cRobot(cRobot2d.GetX(), cRobot2d.GetY(), 0.00);
      /* The battery: a percentage text above the robot */
      
      char pchText[32];
      std::snprintf(pchText, sizeof(pchText), "%.0f%%", m_cLoopFunctions.GetBattery());
      DrawText(cRobot + CVector3(0.06, 0.06, 0.1), pchText, CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   REGISTER_QTOPENGL_USER_FUNCTIONS(CEx2QTUserFunctions, "ex2_qt_user_functions")

}
