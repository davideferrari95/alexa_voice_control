cmake_minimum_required(VERSION 3.0.2)
project(alexa_voice_control)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  parameter_msg.msg
	movement_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ${PROJECT_NAME}
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
		include
		lib
	LIBRARIES 
		desired_velocity_qp
	CATKIN_DEPENDS
		message_runtime
)

###########
## Build ##
###########

include_directories(
	include
	lib
  ${catkin_INCLUDE_DIRS}
)

add_library(desired_velocity_qp
	lib/desired_velocity_qp/ldl.c
	lib/desired_velocity_qp/solver.c
	lib/desired_velocity_qp/matrix_support.c
	lib/desired_velocity_qp/testsolver.c
	lib/desired_velocity_qp/util.c
)

add_executable(alexa_voice_control_Node 
	src/Voice_Control/alexa_voice_control_Node.cpp 
	src/Voice_Control/alexa_voice_control.cpp)
add_dependencies(alexa_voice_control_Node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(alexa_voice_control_Node 
	${catkin_LIBRARIES}
	desired_velocity_qp
)

#############
## Install ##
#############

install(PROGRAMS 

	src/Alexa_TTS/alexa_tts.py 
	src/Skill_Server/skill_server.py
	src/Skill_Server/skill_server_launcher.py
		
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
