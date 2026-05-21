cmake_minimum_required(VERSION 3.5)
project(icp_relocalization)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_BUILD_TYPE Release)

set(USE_LIVOX 1)
# define use_livox
if(USE_LIVOX)
  add_definitions(-DUSE_LIVOX)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration search kdtree)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3)
if(USE_LIVOX)
  find_package(livox_ros_driver2 REQUIRED)
endif()

include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Ubuntu 22.04: PCL 可能引用不存在的 /usr/lib/libOpenNI2.so，实际在 x86_64-linux-gnu 下
set(ICP_PCL_LIBS
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_REGISTRATION_LIBRARIES}
  ${PCL_SEARCH_LIBRARIES}
  ${PCL_KDTREE_LIBRARIES}
)
find_library(OPENNI2_LIB NAMES OpenNI2 PATHS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
if(OPENNI2_LIB)
  list(APPEND ICP_PCL_LIBS ${OPENNI2_LIB})
endif()

add_executable(icp_node src/icp_node.cpp)
find_library(LIBUSB_1 NAMES usb-1.0 PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu REQUIRED)
target_link_libraries(icp_node ${ICP_PCL_LIBS} ${LIBUSB_1})
ament_target_dependencies(icp_node rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen pcl_conversions)
if(USE_LIVOX)
  ament_target_dependencies(icp_node livox_ros_driver2)
endif()

add_executable(transform_publisher src/transform_publisher.cpp)
ament_target_dependencies(transform_publisher rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install launch files
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS icp_node transform_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
