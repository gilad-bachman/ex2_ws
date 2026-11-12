#ifndef EX2_BATTERY_HPP
#define EX2_BATTERY_HPP

/*
 * Exercise 2: the robot's battery. DO NOT MODIFY.
 *
 * The simulated Pi-puck has no battery, so the referee (the loop functions)
 * runs one for it. Every step, just before your ControlStep(), the referee
 * writes the battery into your controller. Your controller gets it by
 * inheriting from CBatteryEquipped (Controller2 already does), and reads it:
 *
 *    Battery().Level()   the level, in percent: 100 is full, 0 is dead
 *    Battery().Home()    where the charger is: the robot's start position
 *
 * The rule, every second of simulated time:
 *   - the battery loses DRAIN_PER_SECOND percent, moving or not;
 *   - while the center of the robot is within CHARGER_RADIUS of the charger,
 *     it also gains CHARGE_PER_SECOND percent.
 * It starts full (100%) and stays between 0 and 100. When it reaches 0 the
 * robot is dead and the run ends.
 */

#include <argos3/core/utility/math/vector2.h>
#include <algorithm>

namespace argos {

   class CEx2LoopFunctions;

   class CBattery {

   public:

      /* Percent lost per second of simulated time */
      static constexpr Real DRAIN_PER_SECOND = 0.8;
      /* Percent gained per second while on the charger */
      static constexpr Real CHARGE_PER_SECOND = 5.0;
      /* The robot is on the charger when its center is this close to Home(), in meters */
      static constexpr Real CHARGER_RADIUS = 0.15;

      /* The battery level, in percent: 100 is full, 0 is dead */
      Real Level() const { return m_fLevel; }

      /* Where the charger is: the robot's start position */
      const CVector2& Home() const { return m_cHome; }

      /* Is a robot whose center is at c_position on the charger? */
      bool IsCharging(const CVector2& c_position) const {
         return (c_position - m_cHome).Length() < CHARGER_RADIUS;
      }

   private:

      /* Only the referee runs the battery */
      friend class CEx2LoopFunctions;

      /* A full battery, with the charger at c_home */
      void Reset(const CVector2& c_home) {
         m_cHome = c_home;
         m_fLevel = 100.0;
      }

      /* The rule, for f_dt seconds with the robot's center at c_position */
      void Step(const CVector2& c_position, Real f_dt) {
         m_fLevel -= DRAIN_PER_SECOND * f_dt;
         if (IsCharging(c_position)) m_fLevel += CHARGE_PER_SECOND * f_dt;
         m_fLevel = std::clamp(m_fLevel, 0.0, 100.0);
      }

      CVector2 m_cHome;
      Real m_fLevel = 100.0;
   };

   /*
    * A controller with a battery. Inherit from it next to CCI_Controller:
    *    class Controller2 : public CCI_Controller, public CBatteryEquipped
    * The referee finds the battery through it, and writes it every step.
    */
   class CBatteryEquipped {

   public:

      virtual ~CBatteryEquipped() = default;

      /* The battery, as the referee left it just before this ControlStep() */
      const CBattery& Battery() const { return m_cBattery; }

   private:

      friend class CEx2LoopFunctions;

      CBattery m_cBattery;
   };
}

#endif
