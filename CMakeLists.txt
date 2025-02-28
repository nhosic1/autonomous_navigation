cmake_minimum_required(VERSION 3.8)
project(autonomous_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_RPI "Build ROS 2 package for Raspberry Pi" OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

if(BUILD_RPI)
  pkg_check_modules(libcamera REQUIRED libcamera)
endif()

include_directories(
  include
)

add_library(stereo_processing SHARED src/stereo_processing.cpp)
ament_target_dependencies(stereo_processing OpenCV)

add_library(localization SHARED src/localization.cpp)
ament_target_dependencies(localization OpenCV)

if(NOT BUILD_RPI)
  add_library(pure_pursuit SHARED src/pure_pursuit.cpp)
  add_library(pid_controller SHARED src/pid_controller.cpp)
endif()

add_executable(autonomous_navigator src/autonomous_navigator.cpp)
target_link_libraries(autonomous_navigator stereo_processing localization)
target_include_directories(autonomous_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(autonomous_navigator PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(autonomous_navigator rclcpp std_msgs sensor_msgs nav_msgs message_filters cv_bridge OpenCV ament_index_cpp image_transport tf2 tf2_geometry_msgs)

if(BUILD_RPI)
  add_executable(frame_publisher src/frame_publisher.cpp)
  target_include_directories(frame_publisher PUBLIC ${libcamera_INCLUDE_DIRS})
  target_link_libraries(frame_publisher ${libcamera_LINK_LIBRARIES})
  target_compile_features(frame_publisher PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(frame_publisher rclcpp sensor_msgs image_transport)
endif()

add_executable(stereo_camera_calibrator src/stereo_camera_calibrator.cpp)
target_compile_features(stereo_camera_calibrator PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(stereo_camera_calibrator rclcpp OpenCV)

add_executable(stereo_viewer src/stereo_viewer.cpp)
target_link_libraries(stereo_viewer stereo_processing)
target_include_directories(stereo_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stereo_viewer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(stereo_viewer rclcpp std_msgs sensor_msgs message_filters cv_bridge OpenCV ament_index_cpp image_transport)

add_executable(state_monitor src/state_monitor.cpp)
target_link_libraries(state_monitor stereo_processing)
target_include_directories(state_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(state_monitor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(state_monitor rclcpp std_msgs sensor_msgs cv_bridge OpenCV image_transport)

if (NOT BUILD_RPI)
  add_executable(path_follower src/path_follower.cpp)
  target_link_libraries(path_follower pure_pursuit pid_controller)
  target_include_directories(path_follower PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(path_follower PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(path_follower rclcpp std_msgs sensor_msgs nav_msgs message_filters OpenCV tf2)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
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

install(TARGETS autonomous_navigator DESTINATION lib/${PROJECT_NAME})
install(TARGETS stereo_camera_calibrator DESTINATION lib/${PROJECT_NAME})
install(TARGETS stereo_viewer DESTINATION lib/${PROJECT_NAME})
install(TARGETS state_monitor DESTINATION lib/${PROJECT_NAME})

if(BUILD_RPI)
  install(TARGETS frame_publisher DESTINATION lib/${PROJECT_NAME})
endif()

if(NOT BUILD_RPI)
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

  install(TARGETS path_follower DESTINATION lib/${PROJECT_NAME})

  ament_python_install_package(${PROJECT_NAME})

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

ament_package()
