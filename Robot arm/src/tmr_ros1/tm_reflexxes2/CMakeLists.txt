cmake_minimum_required(VERSION 2.8.3)
project(tm_reflexxes2)

set(BOOST_FLAG TRUE)
if(BOOST_FLAG)
    add_definitions( -DUSE_BOOST )
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  std_msgs
  tm_msgs
  tm_driver
  tm_kinematics
  geometry_msgs
  message_filters
)

LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-2.8/Modules")
find_package(Eigen3 REQUIRED Dense QR)
find_package(Boost REQUIRED COMPONENTS system thread chrono)


if(BOOST_FLAG)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES TM_REFLEXXESII_LIB RMLTYPEII_LIB   
    CATKIN_DEPENDS roscpp sensor_msgs tm_driver tm_kinematics geometry_msgs std_msgs message_filters 
    DEPENDS boost
)
else()
catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES TM_REFLEXXESII_LIB RMLTYPEII_LIB   
    CATKIN_DEPENDS roscpp sensor_msgs tm_driver tm_kinematics geometry_msgs std_msgs message_filters 
)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
	include
  /home/tm5/Documents/tm5/catkin_ws/devel/include
	ReflexxesTypeII/include
    ${catkin_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIRS}
	)

#############
## Build   ##
#############


file(GLOB RMLtypeII_DIR ReflexxesTypeII/src/*.cpp)
file(GLOB tm_reflexxexII_DIR src/tm_reflexxes.cpp)

add_library(RMLTYPEII_LIB ${RMLtypeII_DIR})
target_link_libraries(RMLTYPEII_LIB 
    ${catkin_LIBRARIES})

add_library(TM_REFLEXXESII_LIB ${tm_reflexxexII_DIR})
target_link_libraries(TM_REFLEXXESII_LIB 
    RMLTYPEII_LIB 
    # TM_modern_driver
    ${catkin_LIBRARIES})

add_library(TM_OTG_LIB src/tm_otg.cpp)
target_link_libraries(TM_OTG_LIB 
    RMLTYPEII_LIB 
    TM_REFLEXXESII_LIB
    ${catkin_LIBRARIES})


# add_executable(tm_traj src/tm_traj.cpp)
# target_link_libraries(tm_traj
#     TM_REFLEXXESII_LIB
#     RMLTYPEII_LIB
#     TM_kinematics
#     TM_modern_driver
#     ${catkin_LIBRARIES}
# )

# add_executable(tm_action src/tm_action.cpp)
# target_link_libraries(tm_action
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_action_gripper src/tm_action_gripper.cpp)
# target_link_libraries(tm_action_gripper
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_action_picking src/tm_action_picking.cpp)
# target_link_libraries(tm_action_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# add_executable(tm_demo_picking_blue_plate src/tm_demo_picking_blue_plate.cpp)
# target_link_libraries(tm_demo_picking_blue_plate
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   # TM_modern_driver
#   TM_print
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# add_executable(tm_yellow_action_picking src/tm_yellow_action_picking.cpp)
# target_link_libraries(tm_yellow_action_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# add_executable(tm_yellow_safe_picking src/tm_yellow_safe_picking.cpp)
# target_link_libraries(tm_yellow_safe_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_table_action_picking src/tm_table_action_picking.cpp)
# target_link_libraries(tm_table_action_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# add_executable(tm_part1_yellow_picking src/tm_part1_yellow_picking.cpp)
# target_link_libraries(tm_part1_yellow_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# add_executable(tm_part1_table_picking src/tm_part1_table_picking.cpp)
# target_link_libraries(tm_part1_table_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_action_picking_safe src/tm_action_picking_safe.cpp)
# target_link_libraries(tm_action_picking_safe
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_action_DLpicking_safe src/tm_action_DLpicking_safe.cpp)
# target_link_libraries(tm_action_DLpicking_safe
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_sim src/tm_sim.cpp)
# target_link_libraries(tm_sim
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_sim_simple src/tm_sim_simple.cpp)
# target_link_libraries(tm_sim_simple
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_sim_gripper src/tm_sim_gripper.cpp)
# target_link_libraries(tm_sim_gripper
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_sim_picking src/tm_sim_picking.cpp)
# target_link_libraries(tm_sim_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# add_executable(tm_sim_yellow_picking src/tm_sim_yellow_picking.cpp)
# target_link_libraries(tm_sim_yellow_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# add_executable(tm_sim_table_picking src/tm_sim_table_picking.cpp)
# target_link_libraries(tm_sim_table_picking
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_sim_picking_safe src/tm_sim_picking_safe.cpp)
# target_link_libraries(tm_sim_picking_safe
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_sim_table_safe src/tm_sim_table_safe.cpp)
# target_link_libraries(tm_sim_table_safe
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_sim_picking_vel src/tm_sim_picking_vel.cpp)
# target_link_libraries(tm_sim_picking_vel
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(tm_cc_gripper src/tm_cc_gripper.cpp)
# target_link_libraries(tm_cc_gripper
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_OTG_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_executable(points_and_lines src/points_and_lines.cpp)
# target_link_libraries(points_and_lines ${catkin_LIBRARIES})

# add_executable(tm_auo src/tm_auo.cpp)
# target_link_libraries(tm_auo
#   TM_REFLEXXESII_LIB
#   RMLTYPEII_LIB
#   TM_OTG_LIB
#   TM_kinematics
#   TM_modern_driver
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

#############
## Install ##
#############

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ReflexxesTypeII/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

