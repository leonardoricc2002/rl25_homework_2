cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================================================
#               TROVA LE DIPENDENZE
# ================================================
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ================================================
#        GENERAZIONE MESSAGGI / AZIONI
# ================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LinearTraj.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ================================================
#               INCLUDE DIRECTORY
# ================================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ================================================
#               NODO PRINCIPALE (SERVER)
# ================================================
add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)

target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node 
  orocos_kdl
  rclcpp
  rclcpp_action
  kdl_parser
  sensor_msgs
  std_msgs
  geometry_msgs
)

target_link_libraries(ros2_kdl_node
  ${orocos_kdl_LIBRARIES}
)

rosidl_target_interfaces(ros2_kdl_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ros2_kdl_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================================================
#               CLIENT ACTION
# ================================================
add_executable(linear_traj_client
  src/linear_traj_client.cpp
)

target_include_directories(linear_traj_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(linear_traj_client PUBLIC cxx_std_17)

ament_target_dependencies(linear_traj_client
  rclcpp
  rclcpp_action
  geometry_msgs
)

rosidl_target_interfaces(linear_traj_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS linear_traj_client
  DESTINATION lib/${PROJECT_NAME}
)

# ================================================
#          INSTALLAZIONE FILE CONFIG E LAUNCH
# ================================================
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# ================================================
#                 TEST E LINTER
# ================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
