cmake_minimum_required(VERSION 3.0.2)
project(teleop_twist_keyboard_plus)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teleop_twist_keyboard_plus
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/yaml-cpp
)

add_library(teleop_twist_keyboard_plus_lib src/teleop_twist_keyboard_plus.cc)
target_link_libraries(teleop_twist_keyboard_plus_lib ${catkin_LIBRARIES} yaml-cpp)

add_executable(teleop_twist_keyboard_plus_node src/teleop_twist_keyboard_plus_node.cc)
target_link_libraries(teleop_twist_keyboard_plus_node teleop_twist_keyboard_plus_lib ${catkin_LIBRARIES})

install(TARGETS teleop_twist_keyboard_plus_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
