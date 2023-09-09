cmake_minimum_required(VERSION 3.8)
project(kinect-freenect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
# pkg_check_modules(LIBFREENECT REQUIRED freenect)
# find_package(libfreenect REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories("/usr/include/libusb-1.0/")

add_executable(kinect_node src/kinect-v1.cpp)
ament_target_dependencies(kinect_node rclcpp sensor_msgs)

target_include_directories(kinect_node PRIVATE
  /usr/include/aarch64-linux-gnu/
  /usr/include/
  ${sensor_msgs_INCLUDE_DIRS} # Add this line
)

target_link_directories(kinect_node PRIVATE
  /usr/lib/aarch64-linux-gnu/
  /usr/lib/
)

target_link_libraries(kinect_node
  freenect
)

install(TARGETS
  kinect_node
  DESTINATION lib/${PROJECT_NAME})

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
