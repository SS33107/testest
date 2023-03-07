cmake_minimum_required(VERSION 2.8.3)
project(tm_kinematics)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
)

#find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES TM_kinematics 
  CATKIN_DEPENDS roscpp 
  #DEPENDS boost
)
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")


###########
## Build ##
###########

#include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(TM_kinematics src/tm_kin.cpp src/tm_jacob.cpp)
set_target_properties(TM_kinematics PROPERTIES COMPILE_DEFINITIONS "TM900_DH_PARAMS")

add_executable(tm_jacob src/main.cpp)
target_link_libraries(tm_jacob ${catkin_LIBRARIES} TM_kinematics)


#############
## Install ##
#############

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark executables and/or libraries for installation
 install(TARGETS tm_jacob TM_kinematics 
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

