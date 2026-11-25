# Exercise 2 - ID: 211397054

set(SRC
  controller2.hpp
  controller2.cpp
  behavior_tree.hpp
  blackboard.hpp
  actuators.hpp
  actuators.cpp
  leaves.hpp
  leaves.cpp
  mission.hpp
  mission.cpp
  # Add other controller source files here
)

add_library(pipuck_controller2 MODULE ${SRC})

target_link_libraries(pipuck_controller2
  PUBLIC
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_pipuck

)

target_include_directories(pipuck_controller2
  PUBLIC
    ${ARGOS_INCLUDE_DIRS}
    # battery.hpp, light_order.hpp
    ${CMAKE_SOURCE_DIR}/sensors
)

set_target_properties(pipuck_controller2 PROPERTIES
  OUTPUT_NAME pipuck_controller2
)
