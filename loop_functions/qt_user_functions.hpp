#ifndef EX2_QT_USER_FUNCTIONS_H
#define EX2_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

namespace argos {

   class CEx2LoopFunctions;

   /*
    * Draws, in the ARGoS window:
    *   - around each light, the ring where a visit counts (15 cm); thick once visited,
    *   - the charger, a yellow ring (15 cm, where it charges and where coming home counts); thick when the robot is home at the end,
    *   - the referee's battery level, above the robot.
    *
    * DO NOT MODIFY.
    */
   class CEx2QTUserFunctions : public CQTOpenGLUserFunctions {

   public:

      CEx2QTUserFunctions();

      void DrawInWorld() override;

   private:

      CEx2LoopFunctions& m_cLoopFunctions;
   };
}

#endif
