##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(robosem_bridge)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED COMPONENTS
    roslint message_generation std_msgs)

catkin_package()
catkin_python_setup()

file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_MODULE_SOURCES} ${${PROJECT_NAME}_SCRIPT_SOURCES})

## Generate msgs in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  PlayTts.msg
  RobotMotion.msg
  TouchSensorEvent.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs 
)


##############################################################################
# Installs
##############################################################################
install(
    PROGRAMS 
        scripts/bridge_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
