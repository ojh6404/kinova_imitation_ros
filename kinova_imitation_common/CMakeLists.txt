cmake_minimum_required(VERSION 2.8.3)
project(kinova_imitation_common)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
