cmake_minimum_required(VERSION 2.8.3)
project(kinova_imitation_common_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  jsk_recognition_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  ClassificationResults.msg
  HumanSkeleton.msg
  HumanSkeletonArray.msg
  RectWithFeature.msg
  RectWithFeatureArray.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  jsk_recognition_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
