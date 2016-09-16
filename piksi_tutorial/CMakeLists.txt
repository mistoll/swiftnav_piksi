cmake_minimum_required(VERSION 2.8.3)
project(piksi_tutorial)

find_package(catkin REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package()

#############
## Install ##
#############

install(DIRECTORY 
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

