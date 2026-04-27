cmake_minimum_required(VERSION 3.8)
project(autonomous_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_RPI "Build ROS 2 package for Raspberry Pi" OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_localization REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AckermannBackUp.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

if(BUILD_RPI)
  pkg_check_modules(libcamera REQUIRED libcamera)
endif()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(nav2_pure_pursuit_controller SHARED src/nav2/nav2_pure_pursuit_controller.cpp)
target_link_libraries(nav2_pure_pursuit_controller pure_pursuit)
target_include_directories(nav2_pure_pursuit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_pure_pursuit_controller
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

add_library(nav2_ackermann_back_up_behavior SHARED src/nav2/nav2_ackermann_back_up_behavior.cpp)
target_include_directories(nav2_ackermann_back_up_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_ackermann_back_up_behavior
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
)
target_link_libraries(nav2_ackermann_back_up_behavior "${cpp_typesupport_target}")

add_library(nav2_ackermann_back_up_action_bt_node SHARED src/nav2/nav2_ackermann_back_up_action_node.cpp)
target_compile_definitions(nav2_ackermann_back_up_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(nav2_ackermann_back_up_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_ackermann_back_up_action_bt_node
  geometry_msgs
  nav2_behavior_tree
  rclcpp
  rclcpp_action
)
target_link_libraries(nav2_ackermann_back_up_action_bt_node "${cpp_typesupport_target}")

add_library(stereo_processing SHARED src/perception/stereo_processing.cpp)
target_include_directories(stereo_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(stereo_processing OpenCV)

add_library(localization SHARED src/localization/localization.cpp)
target_include_directories(localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(localization OpenCV)

add_library(pure_pursuit SHARED src/control/pure_pursuit.cpp)
target_include_directories(pure_pursuit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_library(pid_controller SHARED src/control/pid_controller.cpp)
target_include_directories(pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pure_pursuit PUBLIC pid_controller)

add_executable(visual_odom_estimator src/localization/visual_odom_estimator.cpp)
target_link_libraries(visual_odom_estimator stereo_processing localization ${PCL_LIBRARIES})
target_include_directories(visual_odom_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(visual_odom_estimator PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(visual_odom_estimator rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs message_filters cv_bridge OpenCV ament_index_cpp image_transport tf2 tf2_geometry_msgs tf2_ros pcl_conversions)

if(BUILD_RPI)
  add_executable(frame_publisher src/localization/frame_publisher.cpp)
  target_include_directories(frame_publisher PUBLIC ${libcamera_INCLUDE_DIRS})
  target_link_libraries(frame_publisher ${libcamera_LINK_LIBRARIES})
  target_compile_features(frame_publisher PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(frame_publisher rclcpp sensor_msgs image_transport)
endif()

add_executable(stereo_camera_calibrator src/perception/stereo_camera_calibrator.cpp)
target_compile_features(stereo_camera_calibrator PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(stereo_camera_calibrator rclcpp OpenCV)

add_executable(stereo_viewer src/perception/stereo_viewer.cpp)
target_link_libraries(stereo_viewer stereo_processing)
target_include_directories(stereo_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stereo_viewer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(stereo_viewer rclcpp std_msgs sensor_msgs message_filters cv_bridge OpenCV ament_index_cpp image_transport)

add_executable(state_monitor src/monitoring/state_monitor.cpp)
target_link_libraries(state_monitor stereo_processing)
target_include_directories(state_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(state_monitor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(state_monitor rclcpp std_msgs sensor_msgs cv_bridge OpenCV image_transport)

add_executable(localization_initializer src/localization/localization_initializer.cpp)
target_include_directories(localization_initializer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(localization_initializer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(localization_initializer
  geometry_msgs
  nav2_msgs
  rclcpp
  robot_localization
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_executable(wheel_odom_estimator src/localization/wheel_odom_estimator.cpp)
target_include_directories(wheel_odom_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(wheel_odom_estimator PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(wheel_odom_estimator rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(gazebo_odom_aligner src/localization/gazebo_odom_aligner.cpp)
target_include_directories(gazebo_odom_aligner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gazebo_odom_aligner PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(gazebo_odom_aligner rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(joint_controller src/control/joint_controller.cpp)
target_include_directories(joint_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joint_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(joint_controller rclcpp std_msgs sensor_msgs nav_msgs)

add_executable(nav_cmd_gate src/control/nav_cmd_gate.cpp)
target_include_directories(nav_cmd_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav_cmd_gate PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(nav_cmd_gate
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_srvs)

add_executable(sensor_message_normalizer src/perception/sensor_message_normalizer.cpp)
target_include_directories(sensor_message_normalizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_message_normalizer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(sensor_message_normalizer rclcpp sensor_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY plugins DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)
install(TARGETS stereo_processing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS nav2_pure_pursuit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS nav2_ackermann_back_up_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS nav2_ackermann_back_up_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS pid_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS pure_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS visual_odom_estimator DESTINATION lib/${PROJECT_NAME})
install(TARGETS stereo_camera_calibrator DESTINATION lib/${PROJECT_NAME})
install(TARGETS stereo_viewer DESTINATION lib/${PROJECT_NAME})
install(TARGETS state_monitor DESTINATION lib/${PROJECT_NAME})
install(TARGETS localization_initializer DESTINATION lib/${PROJECT_NAME})
install(TARGETS wheel_odom_estimator DESTINATION lib/${PROJECT_NAME})
install(TARGETS gazebo_odom_aligner DESTINATION lib/${PROJECT_NAME})
install(TARGETS joint_controller DESTINATION lib/${PROJECT_NAME})
install(TARGETS nav_cmd_gate DESTINATION lib/${PROJECT_NAME})
install(TARGETS sensor_message_normalizer DESTINATION lib/${PROJECT_NAME})

if(BUILD_RPI)
  install(TARGETS frame_publisher DESTINATION lib/${PROJECT_NAME})
endif()

if(NOT BUILD_RPI)
  install(PROGRAMS
    scripts/state_monitor.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core plugins/nav2_pure_pursuit_controller_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core plugins/nav2_ackermann_back_up_behavior_plugin.xml)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
