cmake_minimum_required(VERSION 2.8.3)
project(scanner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


add_executable(pcl_processing src/pcl_processing.cpp)
target_link_libraries(pcl_processing ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(kinect2_emulator src/kinect2_emulator.cpp)
target_link_libraries(kinect2_emulator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(showRegResultOnline src/showRegResultOnline.cpp)
target_link_libraries(showRegResultOnline ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(setup src/setup.cpp)
target_link_libraries(setup ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(rotation_dir src/rotation_dir.cpp)
target_link_libraries(rotation_dir ${catkin_LIBRARIES} ${PCL_LIBRARIES})



