cmake_minimum_required(VERSION 2.8.3)
project(tmc_vision_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs)

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
  Label.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    message_runtime
    std_msgs
)
