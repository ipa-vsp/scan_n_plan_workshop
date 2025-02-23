cmake_minimum_required(VERSION 3.5)
project(snp_motion_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS2_MOVEIT_DEPS
  rclcpp
  snp_msgs
  std_srvs
  snp_msgs
  std_srvs
  tf2_eigen
  Eigen3
  Boost
  eigen3_cmake_module
  moveit_ros_planning
  moveit_visual_tools
  geometric_shapes
)

find_package(ament_cmake REQUIRED)

foreach(dep ${ROS2_MOVEIT_DEPS})
  find_package(${dep} REQUIRED)
endforeach()


# find_package(descartes_light REQUIRED)
find_package(LAPACK REQUIRED) # Requried for ikfast
find_package(yaml-cpp REQUIRED)

# MoveIt Plannint Server
add_executable(${PROJECT_NAME}_moveit_node src/moveit_planning_server.cpp)
target_include_directories(
  ${PROJECT_NAME}_moveit_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_moveit_node ${ROS2_MOVEIT_DEPS})

# Install the executable(s)
install(TARGETS ${PROJECT_NAME}_moveit_node DESTINATION lib/${PROJECT_NAME})

# Install the header(s)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${ROS2_MOVEIT_DEPS})
# ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)
