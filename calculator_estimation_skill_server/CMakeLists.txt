cmake_minimum_required(VERSION 3.0.2)
project(calculator_estimation_skill_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  calculator_estimation_skill_msgs
  message_generation
  std_msgs
)
find_package(PCL REQUIRED)


file(GLOB PROGRAM_HEADERS RELATIVE ${PROJECT_SOURCE_DIR} "include/${PROJECT_NAME}/*.h")



catkin_package(
               INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(common
            src/common/verbosity_levels.cpp
            )

add_library(${PROJECT_NAME}
            src/calculator_estimation_skill_server.cpp
            #and all child classes.cpp
            src/calculator_estimation_base.cpp
            )

## Set dependencies of the library
target_link_libraries(common
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      )

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      common
                      )

## Define a executable
add_executable(${PROJECT_NAME}_node
               src/calculator_estimation_skill_server_node.cpp)

## Define the executable dependencies
target_link_libraries(${PROJECT_NAME}_node
                      common
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${PROJECT_NAME})




