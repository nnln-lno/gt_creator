cmake_minimum_required(VERSION 3.8)
project(gt_creator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uwb_driver REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(gt_creator_lib SHARED
  src/gt_creator.cpp
)

add_executable(gt_creator_node src/main.cpp)

ament_target_dependencies(gt_creator_node
  rclcpp
  std_msgs
  geometry_msgs
  uwb_driver
)

ament_target_dependencies(gt_creator_lib
  rclcpp
  std_msgs
  geometry_msgs
  uwb_driver
)

target_link_libraries(gt_creator_node
  gt_creator_lib
  ${PCL_LIBRARIES}
)

install(TARGETS
  gt_creator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gt_creator_lib
  EXPORT gt_creator_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include 
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
