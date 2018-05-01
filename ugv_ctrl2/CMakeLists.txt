cmake_minimum_required(VERSION 2.8.3)
project(ugv_ctrl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ugv_ctrl
#  CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS system_lib 
 CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ugv_ctrl_node src/main.cpp) 

target_link_libraries(ugv_ctrl_node
   ${catkin_LIBRARIES}
   pthread
)

add_dependencies(ugv_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ugv_ctrl_node
  ${catkin_LIBRARIES}
)
