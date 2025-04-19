cmake_minimum_required(VERSION 3.0.2)
project(mars)


 find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              rospy 
              std_msgs 
              geometry_msgs
              message_generation
              message_filters
              sensor_msgs)



## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   IRStamped.msg
 )

## Generate services in the 'srv' folder
#  add_service_files(
#     FILES
#     convert_pose.srv
#  )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mars
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
#########
##Build##
#########

include_directories(include ${catkin_INCLUDE_DIRS})



# add_executable(ir_positions src/ir_positions.cpp)
# target_link_libraries(ir_positions ${catkin_LIBRARIES})
# #add_dependencies(ir_position ir_positions_generates_messages_gencpp)

add_executable(sensor_fusion src/sensor_fusion.cpp)
target_link_libraries(sensor_fusion ${catkin_LIBRARIES})
