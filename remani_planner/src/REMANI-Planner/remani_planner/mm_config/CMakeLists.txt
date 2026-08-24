cmake_minimum_required(VERSION 2.8.3)
project(mm_config)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  visualization_msgs
  plan_env
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mm_config 
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

#add_message_files{
#    FILES
#    Assignment.msg
#}

add_library( mm_config
    src/mm_config.cpp
    )

target_link_libraries( mm_config
    ${catkin_LIBRARIES} 
    ) 

# ################################
# CMake: CR10 FK validation node begin
# ################################
add_executable(test_cr10_fk src/test_cr10_fk.cpp)
target_link_libraries(test_cr10_fk mm_config ${catkin_LIBRARIES})
# ################################
# CMake: CR10 FK validation node end
# ################################

# ################################
# CMake: EE kinematics API compile check begin
# ################################
add_executable(test_ee_kinematics_compile src/test_ee_kinematics_compile.cpp)
target_link_libraries(test_ee_kinematics_compile mm_config ${catkin_LIBRARIES})
# ################################
# CMake: EE kinematics API compile check end
# ################################

# ################################
# CMake: EE pose and Jacobian Milestone A tests begin
# ################################
add_executable(test_ee_pose_ik src/test_ee_pose_ik.cpp)
target_link_libraries(test_ee_pose_ik mm_config ${catkin_LIBRARIES})
# ################################
# CMake: EE pose and Jacobian Milestone A tests end
# ################################

