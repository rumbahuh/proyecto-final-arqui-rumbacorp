cmake_minimum_required(VERSION 3.8)
project(pfa_restaurante)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED) # Esto lo he añadido del bumpgo
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED) # Esto lo he añadido del bumpgo

set(dependencies
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  vision_msgs
  ament_index_cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/controlper/ControlNode.cpp
  src/controlper/PIDControler.cpp
  src/controlper/TfPublisher.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(control_main src/control_main.cpp)
ament_target_dependencies(control_main ${dependencies})
target_link_libraries(control_main ${PROJECT_NAME})

add_executable(tf_main src/tf_main.cpp)
ament_target_dependencies(tf_main ${dependencies})
target_link_libraries(tf_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  control_main
  tf_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
) # Esto lo he añadido del bumpgo pero no sé si es en lugar del launch

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
