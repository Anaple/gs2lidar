cmake_minimum_required(VERSION 3.8)

project(gs2lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


# target_link_libraries(ydlidar_driver rt pthread)
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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src/sdk)

set(CMAKE_BUILD_TYPE Release)

FILE(GLOB SDK_SRC
  "src/sdk/*.cpp"
  "src/sdk/*.c"
  "src/sdk/*.h"
  "src/sdk/impl/unix/*.cpp"
  "src/sdk/impl/unix/*.h"
)

add_library(ydlidar_driver STATIC ${SDK_SRC})

# Link gs2lidar executable with ydlidar_driver library
add_executable(gs2lidar
  src/main_ros2node.cpp
)
target_link_libraries(gs2lidar ydlidar_driver)

# add_executable(gs2demo
#   src/main.cpp
# )
# target_link_libraries(gs2demo ydlidar_driver)

ament_target_dependencies(gs2lidar rclcpp sensor_msgs tf2_ros)

add_executable(lidar_dis
  src/main_getdistance.cpp
)
target_link_libraries(lidar_dis ydlidar_driver)
ament_target_dependencies(lidar_dis geometry_msgs rclcpp sensor_msgs tf2_ros)

install(TARGETS gs2lidar
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS lidar_dis
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
