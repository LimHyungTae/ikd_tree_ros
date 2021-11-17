cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ikd_tree_ros)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    nav_msgs
    roslaunch
    cv_bridge
    pcl_conversions
    pcl_ros
    geometry_msgs
    laser_geometry
    sensor_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)


find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)


link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(delete_demo src/delete_test.cpp include/ikd_Tree.cpp)
target_link_libraries(delete_demo ${PCL_LIBRARIES})

add_executable(pcl_delete_demo src/pcd_delete_test.cpp include/ikd_Tree.cpp)
target_link_libraries(pcl_delete_demo ${PCL_LIBRARIES})

add_executable(ikd_tree_demo src/ikd_Tree_demo.cpp include/ikd_Tree.cpp)
target_link_libraries(ikd_tree_demo ${PCL_LIBRARIES})

add_executable(speed_demo src/pcd_speed_demo.cpp include/ikd_Tree.cpp)
target_link_libraries(speed_demo ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(speed_demo ikd_tree_ros_generate_messages_cpp)

add_executable(viz_demo src/deprecated_pcd_viz_demo.cpp include/ikd_Tree.cpp)
target_link_libraries(viz_demo ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(viz_demo ikd_tree_ros_generate_messages_cpp)