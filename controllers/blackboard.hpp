/*
 * Exercise 2 - ID: 211397054
 *
 * The blackboard: what the robot knows, shared by every node in the tree.
 * Controller2 writes this step's sensor readings into it before each tick;
 * the nodes read them, and keep the mission's memory here.
 */
#ifndef BLACKBOARD_HPP
#define BLACKBOARD_HPP

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <array>
#include <vector>

#include "light_order.hpp"

namespace argos {

   struct SBlackboard {

      /* ---- This step's sensor readings (written by Controller2) ---- */

      /* Positioning: where the robot is, and which way it faces */
      CVector2 Position;
      CRadians Heading;

      /* Rangefinders: distance in meters for labels 0..7 (up to 0.2) */
      std::array<Real, 8> Proximity{};

      /* Camera: the blobs it sees (angle, distance in cm, color) */
      std::vector<CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob> Blobs;

      /* Battery: level in percent, and where the charger is */
      Real BatteryLevel = 100.0;
      CVector2 Home;

      /* Clock, in seconds */
      Real Time = 0.0;

      /* ---- The mission ---- */

      /* The lights, in the order to visit them */
      std::vector<SLightInfo> Order;

      /* ---- The mission's memory ---- */

      /*
       * The lights, by their index in Order (0 = the first to visit).
       * Known/Position: written by Controller2 whenever the camera sees the
       * light, whichever leaf is running; the latest sighting wins.
       * Visited: written by the Visit leaf.
       */
      std::array<bool, 3> LightKnown{};
      std::array<CVector2, 3> LightPosition{};   // world frame, in m
      std::array<bool, 3> LightVisited{};
   };

}

#endif
