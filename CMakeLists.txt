cmake_minimum_required(VERSION 3.8)
project(multi_object_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tracking_msgs REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS highgui)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(kitti_publisher_node
  src/kitti_publisher_node.cpp
  src/kitti_publisher.cpp
)

ament_target_dependencies(kitti_publisher_node
  rclcpp
  cv_bridge
  pcl_conversions
  sensor_msgs
  std_msgs
  tracking_msgs
  tf2
)

target_include_directories(kitti_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(kitti_publisher_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

target_compile_features(kitti_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(multi_object_tracking_node
  src/multi_object_tracking_node.cpp
  src/multi_object_tracking.cpp
  src/tracker.cpp
  src/imm_ukf.cpp
  src/hungarianAlg.cpp
  src/ukf.cpp)

ament_target_dependencies(multi_object_tracking_node
  rclcpp
  cv_bridge
  pcl_conversions
  tf2
  sensor_msgs
  std_msgs
  visualization_msgs
  tracking_msgs)

target_include_directories(multi_object_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

target_link_libraries(multi_object_tracking_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

target_compile_features(multi_object_tracking_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS kitti_publisher_node multi_object_tracking_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
