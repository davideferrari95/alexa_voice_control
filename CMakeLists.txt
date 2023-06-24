cmake_minimum_required(VERSION 3.0.2)
project(alexa_voice_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
	# include
  CATKIN_DEPENDS
	# message_runtime
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
    script/Exp_Manager.py
  DESTINATION
  	${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
