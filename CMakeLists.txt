cmake_minimum_required(VERSION 3.0.2)
project(dynamic_avoidance)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  ackermann_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES sensor_msgs)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dynamic_avoidance_node src/dynamic_avoidance.cpp)

#add_dependencies(dynamic_avoidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dynamic_avoidance_node ${catkin_LIBRARIES})
