cmake_minimum_required(VERSION 2.8.3)
project(irobi_bridge)

find_package(catkin REQUIRED COMPONENTS
  rapix_bridge
)

catkin_python_setup()
catkin_package()
