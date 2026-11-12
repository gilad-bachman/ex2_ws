#ifndef EX2_LIGHT_ORDER_HPP
#define EX2_LIGHT_ORDER_HPP

/*
 * Exercise 2: the order of the lights. DO NOT MODIFY.
 *
 * The order is a parameter in the .argos file, in the controller's <params> tag:
 *
 *    <params order="red,green,blue" />
 *
 * Your controller and the referee read the same parameter, and the test runs
 * use other orders, so read it; don't hard-code it. ReadLightOrder() turns it
 * into the three lights, in the order they must be visited:
 *
 *    std::vector<SLightInfo> vecOrder = ReadLightOrder(t_tree);   // in Init()
 *    vecOrder[0].Name    "red"
 *    vecOrder[0].Color   CColor::RED: the color the camera reports for it
 */

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>
#include <sstream>
#include <string>
#include <vector>

namespace argos {

   struct SLightInfo {
      std::string Name;
      CColor Color;
   };

   /* "red,green,blue" -> the three lights, in that order. Exactly red, green and blue, once each */
   inline std::vector<SLightInfo> ParseLightOrder(const std::string& str_order) {
      const std::vector<SLightInfo> vecAll = {
         { "red", CColor::RED }, { "green", CColor::GREEN }, { "blue", CColor::BLUE } };
      std::vector<SLightInfo> vecOrder;
      std::istringstream cIn(str_order);
      for (std::string strName; std::getline(cIn, strName, ',');) {
         strName.erase(0, strName.find_first_not_of(" \t"));
         strName.erase(strName.find_last_not_of(" \t") + 1);
         bool bFound = false;
         for (const SLightInfo& s : vecAll) {
            if (s.Name != strName) continue;
            for (const SLightInfo& sTaken : vecOrder)
               if (sTaken.Name == strName) THROW_ARGOSEXCEPTION("order=\"" << str_order << "\": " << strName << " twice");
            vecOrder.push_back(s);
            bFound = true;
         }
         if (!bFound) THROW_ARGOSEXCEPTION("order=\"" << str_order << "\": unknown light \"" << strName << "\" (red, green or blue)");
      }
      if (vecOrder.size() != 3) THROW_ARGOSEXCEPTION("order=\"" << str_order << "\": needs red, green and blue, once each");
      return vecOrder;
   }

   /* t_params: the controller's <params> tag (the t_tree that Init() gets) */
   inline std::vector<SLightInfo> ReadLightOrder(TConfigurationNode& t_params) {
      std::string strOrder;
      GetNodeAttribute(t_params, "order", strOrder);
      return ParseLightOrder(strOrder);
   }
}

#endif
