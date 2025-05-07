cmake_minimum_required(VERSION 3.8)
project(pfa_restaurante)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(libzmq3-dev)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  behaviortree_cpp_v3
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  vision_msgs
  nav2_msgs
  ament_index_cpp
  audio_common_msgs
  whisper_msgs
  std_msgs
  libzmq3-dev
)

# --- Plugin node libraries ---
add_library(gotodoor_pfa_node SHARED src/controlper/GoToDoor.cpp)
add_library(countpeople_pfa_node SHARED src/controlper/CountPeople.cpp)
add_library(countpeople_pfa_node SHARED src/controlper/CheckTable.cpp)
add_library(countpeople_pfa_node SHARED src/controlper/GoToTable.cpp)

include_directories(include ${CMAKE_SOURCE_DIR}/src)

list(APPEND plugin_libs
  gotodoor_pfa_node
  countpeople_pfa_node
  checktable_pfa_node
  gototable_pfa_node
)

foreach(pfa_plugin ${plugin_libs})
  ament_target_dependencies(${pfa_plugin} ${dependencies})
  target_compile_definitions(${pfa_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# --- Main executable ---
add_executable(control_main src/control_main.cpp)
ament_target_dependencies(control_main ${dependencies})
target_link_libraries(control_main ${ZMQ_LIBRARIES})  # <-- LINK ZMQ

# --- Install targets ---
install(TARGETS
  ${plugin_libs}
  control_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install BT XMLs and launch files
install(DIRECTORY behavior_tree_xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- Export info ---
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
