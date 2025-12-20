cmake_minimum_required(VERSION 3.8)
project(sunhawk_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(SDL2 REQUIRED)

add_executable(keyboard_manual_control_node_sdl2 src/keyboard_manual_control_node_sdl2.cpp)
ament_target_dependencies(keyboard_manual_control_node_sdl2 rclcpp px4_msgs)

add_executable(sensor_combined_listener src/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)

add_executable(engine_rpm_model_node src/engine_rpm_model_node.cpp)
ament_target_dependencies(engine_rpm_model_node rclcpp px4_msgs)

add_executable(debug_vect_advertiser_sim_time src/debug_vect_advertiser_sim_time.cpp)
ament_target_dependencies(debug_vect_advertiser_sim_time rclcpp px4_msgs)

target_include_directories(keyboard_manual_control_node_sdl2 PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(keyboard_manual_control_node_sdl2 ${SDL2_LIBRARIES})

install(TARGETS
  sensor_combined_listener
  engine_rpm_model_node
  debug_vect_advertiser_sim_time
  keyboard_manual_control_node_sdl2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
