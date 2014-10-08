##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(robosem_bridge)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED COMPONENTS
    roslint)

catkin_package()
catkin_python_setup()

file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_MODULE_SOURCES} ${${PROJECT_NAME}_SCRIPT_SOURCES})

##############################################################################
# Installs
##############################################################################
install(
    PROGRAMS 
        scripts/robosem_bridge.py
        scripts/robosem_manager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
