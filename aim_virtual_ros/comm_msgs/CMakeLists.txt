cmake_minimum_required(VERSION 2.8.3)
project(comm_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
add_message_files(
   FILES
   vehicleStatus.msg
   objInfo.msg
   objsInfo.msg
   gpsLocal.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msgs
   CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
