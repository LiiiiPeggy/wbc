cmake_minimum_required(VERSION 2.8.3)
project(remani_planner)

set(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  mm_config
  remani_real_msgs
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES remani_planner
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils mm_config remani_real_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library( vis_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( vis_utils
    ${catkin_LIBRARIES} 
    )  

add_library(candidate_transaction_builder
  src/candidate_transaction_builder.cpp)
target_link_libraries(candidate_transaction_builder
  ${catkin_LIBRARIES})
add_dependencies(candidate_transaction_builder ${catkin_EXPORTED_TARGETS})

add_executable(remani_planner_node
  src/remani_planner_node.cpp 
  src/remani_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(remani_planner_node 
  candidate_transaction_builder
  ${catkin_LIBRARIES}
  vis_utils
  )
add_dependencies(remani_planner_node ${catkin_EXPORTED_TARGETS})

# ################################
# CMake: EE interactive marker node begin
# ################################
add_executable(ee_goal_marker_node
  src/ee_goal_marker_node.cpp
)
target_link_libraries(ee_goal_marker_node
  ${catkin_LIBRARIES}
)
# ################################
# CMake: EE interactive marker node end
# ################################

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_execution_policy test/test_execution_policy.cpp)
  if(TARGET test_execution_policy)
    target_link_libraries(test_execution_policy ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_candidate_transaction_builder
    test/test_candidate_transaction_builder.cpp)
  if(TARGET test_candidate_transaction_builder)
    target_link_libraries(test_candidate_transaction_builder
      candidate_transaction_builder ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_plan_only_policy test/test_plan_only_policy.cpp)
  if(TARGET test_plan_only_policy)
    target_link_libraries(test_plan_only_policy ${catkin_LIBRARIES})
    add_dependencies(test_plan_only_policy ${catkin_EXPORTED_TARGETS})
  endif()

  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS
    test/fake_plan_only_actual_state.py
    test/test_remani_plan_only.py
    test/test_remani_sim_owner.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/remani_plan_only.test)
  add_rostest(test/remani_sim_owner.test)
endif()
