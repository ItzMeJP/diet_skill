cmake_minimum_required(VERSION 3.0.2)
project(calculator_estimation_skill_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
        FILES
        CalculatorResult.msg
)

add_action_files(
  FILES
  CalculatorEstimationSkill.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

