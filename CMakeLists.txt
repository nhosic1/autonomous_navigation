cmake_minimum_required(VERSION 3.8)
project(autonomous_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera)

include_directories(
  include
)

set(library_name stereo_processing)

add_library(${library_name} SHARED src/stereo_processing.cpp)
ament_target_dependencies(${library_name} OpenCV)

function(add_package_executable target)
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} ${library_name})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
endfunction()

add_package_executable(stereo_depth_estimator)
ament_target_dependencies(stereo_depth_estimator rclcpp std_msgs sensor_msgs message_filters cv_bridge OpenCV ament_index_cpp)

add_package_executable(frame_publisher)
ament_target_dependencies(frame_publisher rclcpp sensor_msgs)

target_include_directories(frame_publisher PUBLIC ${libcamera_INCLUDE_DIRS})
target_link_libraries(frame_publisher ${libcamera_LINK_LIBRARIES})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS stereo_depth_estimator DESTINATION lib/${PROJECT_NAME})
install(TARGETS frame_publisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
