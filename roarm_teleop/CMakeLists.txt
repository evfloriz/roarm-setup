cmake_minimum_required(VERSION 3.8)
project(roarm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Curses REQUIRED)

add_executable(teleop src/teleop.cpp)

target_link_libraries(teleop
  ${CURSES_LIBRARIES}
)

ament_target_dependencies(teleop rclcpp std_msgs)

install(TARGETS teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()
