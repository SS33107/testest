cmake_minimum_required(VERSION 3.5)
project(tm_kinematics)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DROS2_BUILD)

#find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3)


###########
## Build ##
###########

#include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${CMAKE_CURRENT_SOURCE_DIR})

add_library(TM_kinematics SHARED src/tm_kin.cpp src/tm_jacob.cpp)
set_target_properties(TM_kinematics PROPERTIES COMPILE_DEFINITIONS "TM900_DH_PARAMS")
ament_target_dependencies(TM_kinematics
        PUBLIC
        rclcpp
        )
target_include_directories(TM_kinematics PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )




#############
## Install ##
#############


ament_export_targets(TM_kinematics HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
)
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS TM_kinematics 
        EXPORT TM_kinematics
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

#############
##  else   ##
#############
add_executable(tm_jacob src/main.cpp)
ament_target_dependencies(tm_jacob 
  rclcpp
)
target_link_libraries(tm_jacob
  TM_kinematics)
install(TARGETS
  tm_jacob
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
