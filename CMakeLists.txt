cmake_minimum_required(VERSION 2.8.3)
project(mavs_ros)

add_definitions(-DUSE_OMP)
add_definitions(-DUSE_EMBREE)

##Modify these to the appropriate paths
#SET(mavs_INCLUDE_DIR  "/home/msucavs/mavs/src/")
#SET(mavs_LIB_DIR  "/home/msucavs/mavs/build/lib")

find_package(MAVS REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  CATKIN_DEPENDS message_runtime tf
)

###########
## Build ##
###########

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}  
  ${PCL_INCLUDE_DIRS} 
  #${mavs_INCLUDE_DIR} 
  #${mavs_INCLUDE_DIR}/thirdparty/reactphysics3d/src
)
  
link_directories(
  ${PCL_LIBRARY_DIRS} 
)

add_executable(mavs_vehicle_node src/mavs_vehicle_node.cpp src/mavs_ros_utils.cpp)
target_link_libraries(mavs_vehicle_node
  ${catkin_LIBRARIES}
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
)

add_executable(mavs_sensors_node src/mavs_sensors_node.cpp src/mavs_ros_utils.cpp)
target_link_libraries(mavs_sensors_node
  ${catkin_LIBRARIES}
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
)

add_executable(sim_manager_node src/sim_manager_node.cpp src/mavs_ros_utils.cpp)
target_link_libraries(sim_manager_node
  ${catkin_LIBRARIES}
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
)

add_executable(mavs_path_planning_node src/mavs_path_planning_node.cpp src/astar.cpp src/mavs_ros_utils.cpp)
target_link_libraries(mavs_path_planning_node
  ${catkin_LIBRARIES}
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
)


## Mark executables for installation
install(TARGETS 
  mavs_vehicle_node
  mavs_sensors_node
  sim_manager_node
  mavs_path_planning_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
