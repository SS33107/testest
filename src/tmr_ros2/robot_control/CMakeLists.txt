cmake_minimum_required(VERSION 3.10)
project(robot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  # set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)

LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-2.8/Modules")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ruckig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tm_msgs REQUIRED)
find_package(tm_kinematics REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR})
# find_package(eigen3_cmake_module)
# if (NOT eigen3_cmake_module_FOUND)
#   return()
# endif()

find_package(Eigen3 REQUIRED)

# include_directories(include ${EIGEN3_INCLUDE_DIR})


set(DEPENDENCIES
    "Eigen3"
)

# list(APPEND include ${Eigen3_INCLUDE_DIRS})

# SET( EIGEN3_INCLUDE_DIR "/usr/include/eigen3" )
# IF( NOT EIGEN3_INCLUDE_DIR )
#     MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
# ENDIF()
# INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)






# add_executable(new_control src/new_control.cpp)
# target_compile_features(new_control PUBLIC cxx_std_17)
# target_link_libraries(new_control ruckig::ruckig)
# ament_target_dependencies(new_control 
#   rclcpp
#   std_msgs
#   sensor_msgs
#   tm_msgs
# )
# install(TARGETS
#   new_control
#   DESTINATION lib/${PROJECT_NAME})


add_executable(gazebo_control src/gazebo_control.cpp)
target_link_libraries(gazebo_control ruckig::ruckig Eigen3::Eigen)
# target_include_directories(gazebo_control PUBLIC "${Eigen3_INCLUDE_DIRS}")
ament_target_dependencies(gazebo_control 
  rclcpp
  std_msgs
  sensor_msgs
  tm_msgs
  tm_kinematics
  ${DEPENDENCIES}
)
install(TARGETS
  gazebo_control
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  ${DEPENDENCIES}
    # eigen3_cmake_module
  )

ament_package()

