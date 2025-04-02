cmake_minimum_required(VERSION 3.0.2)
project(diet_estimation_skill_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  diet_estimation_skill_msgs
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
add_library(mycommons
            src/common/verbosity_levels.cpp
            src/common/std_vector_operations.cpp
            )

add_library(${PROJECT_NAME}
        src/diet_estimation_skill_server.cpp
        src/diet_estimation_base.cpp
            src/heuristics/protein_selector.cpp
            src/heuristics/fiber_selector.cpp
)

## Set dependencies of the library
target_link_libraries(mycommons
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      )

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      mycommons
                      )

## Define a executable
add_executable(${PROJECT_NAME}_node
        src/diet_estimation_skill_server_node.cpp)

## Define the executable dependencies
target_link_libraries(${PROJECT_NAME}_node
                      mycommons
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${PROJECT_NAME})
