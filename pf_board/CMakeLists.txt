cmake_minimum_required(VERSION 2.8.3)
project(pf_board)

## Use C++14
add_definitions(-std=c++14)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    control_msgs
    roslint
    pf_msgs
    hardware_interface
    controller_manager
)

## Find system libraries
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    control_msgs
    pf_msgs
    hardware_interface
    controller_manager
#  DEPENDS
#    Boost
)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/external/vstruct/include
  include/external/generated
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/pf_board/comms/spi_driver.cpp
  src/pf_board/comms/transport_layer.cpp
  src/pf_board/states/sync_state.cpp
  src/pf_board/states/stop_state.cpp
  src/pf_board/states/wait_state.cpp
  src/pf_board/states/error_state.cpp
  src/pf_board/states/run_state.cpp
  src/pf_board/states/non_recoverable_state.cpp
  src/pf_board/pf_board.cpp
  src/pf_board/simple_timer.cpp
  src/pf_board/time_source.cpp
  src/pf_board/pf_board_control.cpp
  src/pf_board_node.cpp
)
## cmd to find
## >find src/ -type f -name "*.cpp"


## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

roslint_cpp()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )

# Mark other files for installation
# install(
#   DIRECTORY doc
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# ## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test
#   test/test_ros_package_template.cpp
#   test/AlgorithmTest.cpp)
# endif()
#
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
# endif()
