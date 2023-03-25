cmake_minimum_required(VERSION 3.0.2)
project(kinova_test)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

#------------------------------------------ user added
set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


option(USE_CONAN "Use the Conan package manager to automatically fetch the Kortex API" ON)


if(UNIX)
  add_definitions(-D_OS_UNIX)
endif()

# Setup Kortex Api Path
if(NOT KORTEX_SUB_DIR)
  set(KORTEX_SUB_DIR "")
else()
  set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
endif()

set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/kortex_api/${KORTEX_SUB_DIR}")

if(CMAKE_BUILD_TYPE EQUAL "Debug")
  set(KORTEX_LIB_SUBDIR "debug")
else()
  set(KORTEX_LIB_SUBDIR "release")
endif()

if(UNIX)
  link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/libKortexApiCpp.a)
elseif(WIN32)
  link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/KortexApiCpp.lib)
endif()

# Add Include Directories
include_directories(${KORTEX_DIR}include)
include_directories(${KORTEX_DIR}include/client)
include_directories(${KORTEX_DIR}include/common)
include_directories(${KORTEX_DIR}include/messages)
include_directories(${KORTEX_DIR}include/client_stubs)

# link other libs
if(UNIX)
  link_libraries(pthread)
endif()
#------------------------------------------- user added


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  kortex_driver
  roscpp
  rospy
  sensor_msgs
  std_msgs
  actionlib
  kdl_parser
)
find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinova_test
#  CATKIN_DEPENDS actionlib_msgs geometry_msgs kortex_driver roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller src/controller.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES} )

#add_executable(velocity_controller src/velocity_controller.cpp)
#add_dependencies(velocity_controller ${catkin_EXPORTED_TARGETS})
#target_link_libraries(velocity_controller ${catkin_LIBRARIES} )

add_executable(high_level_velocity src/high_level_velocity.cpp)
add_dependencies(high_level_velocity ${catkin_EXPORTED_TARGETS})
target_link_libraries(high_level_velocity ${catkin_LIBRARIES} )

add_executable(twist_command src/twist_command.cpp)
add_dependencies(twist_command ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_command ${catkin_LIBRARIES} )

add_executable(kdl_test src/kdl_test.cpp)
add_dependencies(kdl_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(kdl_test ${catkin_LIBRARIES} )


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS src/actuator_config/example_actuator_configuration.py
# 		src/full_arm/example_cartesian_poses_with_notifications.py
# 		src/full_arm/example_full_arm_movement.py
# 		src/full_arm/example_waypoint_action_client.py
# 		src/move_it/example_move_it_trajectories.py
# 		src/vision_config/example_vision_configuration.py
# 		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )

## Mark executables and/or libraries for installation
install(TARGETS
		controller
        high_level_velocity
        twist_command
        kdl_test
#        velocity_controller
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
