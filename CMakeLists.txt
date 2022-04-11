cmake_minimum_required(VERSION 2.8.3)
project(hero_pcl)

set(CMAKE_CXX_STANDARD 17)

include_directories("/usr/include/eigen3")

find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros message_generation std_msgs roscpp dynamic_reconfigure
geometry_msgs
nav_msgs
rosconsole
sensor_msgs
tf
tf2_ros
tf2_geometry_msgs
laser_geometry
gazebo_dev)

find_package(Ceres REQUIRED)
find_package(Boost 1.58 REQUIRED)
find_package(PCL 1.8.1 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
include_directories(src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(FILES MsgTutorial.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hero_pcl
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hero_pcl_test src/hero_pcl_test.cpp)
add_dependencies(hero_pcl_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
target_link_libraries(hero_pcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(hero_ceres_test src/hero_ceres_test.cc)
add_dependencies(hero_ceres_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
target_link_libraries(hero_ceres_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)