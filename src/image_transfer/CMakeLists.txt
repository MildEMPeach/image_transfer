cmake_minimum_required(VERSION 3.8)
project(image_transfer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(image_transfer_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)

add_executable(image_transfer_server src/image_transfer_server.cpp)
add_executable(image_transfer_client src/image_transfer_client.cpp)
ament_target_dependencies(
  image_transfer_server
  rclcpp
  image_transfer_interfaces
# cv_bridge
)

ament_target_dependencies(
  image_transfer_client
  rclcpp
  image_transfer_interfaces
# cv_bridge
  sensor_msgs
)

target_link_libraries(
  image_transfer_server
  ${OpenCV_LIBS}
)

target_link_libraries(
  image_transfer_client
  ${OpenCV_LIBS}
)

install(
  TARGETS image_transfer_server image_transfer_client
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
