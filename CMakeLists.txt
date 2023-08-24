cmake_minimum_required(VERSION 3.0.2)
project(alexa_voice_control)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  VoiceCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    ${PROJECT_NAME}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
	# include
  CATKIN_DEPENDS
    rospy
  	message_runtime
)

###########
## Build ##
###########

include_directories(
	# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(
  PROGRAMS
    script/Skill/backend.py
    script/TTS/node_red.py
  DESTINATION
  	${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
