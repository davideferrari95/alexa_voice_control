cmake_minimum_required(VERSION 3.0.2)
project(alexa_voice_control)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
)


include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS 

	src/Alexa_TTS/alexa_tts.py 
	src/Skill_Server/skill_server.py
	src/Skill_Server/skill_server_launcher.py
		
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

add_executable(alexa_voice_control_Node 
	src/Voice_Control/alexa_voice_control_Node.cpp 
	src/Voice_Control/alexa_voice_control.cpp)
target_link_libraries(alexa_voice_control_Node ${catkin_LIBRARIES})
add_dependencies(alexa_voice_control_Node ${catkin_EXPORTED_TARGETS})
