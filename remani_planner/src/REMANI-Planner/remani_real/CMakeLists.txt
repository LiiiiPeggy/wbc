cmake_minimum_required(VERSION 3.0.2)
project(remani_real)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  quadrotor_msgs
  remani_real_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  mm_controller
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES remani_real_candidate remani_real_state_bridge
  CATKIN_DEPENDS nav_msgs quadrotor_msgs remani_real_msgs roscpp sensor_msgs tf2 tf2_ros mm_controller
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../mm_controller/include
)

add_library(remani_real_candidate
  src/candidate_trajectory.cpp
  src/candidate_assembler.cpp
)
target_compile_features(remani_real_candidate PUBLIC cxx_std_14)
add_dependencies(remani_real_candidate ${catkin_EXPORTED_TARGETS})
target_link_libraries(remani_real_candidate
  ${catkin_LIBRARIES}
)

# ################################
# CMake: State Bridge mapping / velocity libraries begin
# ################################
add_library(remani_real_state_bridge
  src/joint_state_mapper.cpp
  src/velocity_estimator.cpp
)
target_compile_features(remani_real_state_bridge PUBLIC cxx_std_14)
add_dependencies(remani_real_state_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(remani_real_state_bridge
  ${catkin_LIBRARIES}
)

add_executable(remani_state_bridge_node
  src/remani_state_bridge_node.cpp
)
target_compile_features(remani_state_bridge_node PRIVATE cxx_std_14)
add_dependencies(remani_state_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(remani_state_bridge_node
  remani_real_state_bridge
  ${catkin_LIBRARIES}
)
# ################################
# CMake: State Bridge mapping / velocity libraries end
# ################################

install(TARGETS remani_real_candidate remani_real_state_bridge remani_state_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_candidate_sampling
    test/test_candidate_sampling.cpp
  )
  if(TARGET test_candidate_sampling)
    target_compile_features(test_candidate_sampling PRIVATE cxx_std_14)
    target_link_libraries(test_candidate_sampling
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()

  # ################################
  # CMake: CandidateAssembler gtest begin
  # ################################
  catkin_add_gtest(test_candidate_assembler
    test/test_candidate_assembler.cpp
  )
  if(TARGET test_candidate_assembler)
    target_compile_features(test_candidate_assembler PRIVATE cxx_std_14)
    target_link_libraries(test_candidate_assembler
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: CandidateAssembler gtest end
  # ################################

  # ################################
  # CMake: State Bridge unit/rostests begin
  # ################################
  catkin_add_gtest(test_joint_state_mapper
    test/test_joint_state_mapper.cpp
  )
  if(TARGET test_joint_state_mapper)
    target_compile_features(test_joint_state_mapper PRIVATE cxx_std_14)
    target_link_libraries(test_joint_state_mapper
      remani_real_state_bridge
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_velocity_estimator
    test/test_velocity_estimator.cpp
  )
  if(TARGET test_velocity_estimator)
    target_compile_features(test_velocity_estimator PRIVATE cxx_std_14)
    target_link_libraries(test_velocity_estimator
      remani_real_state_bridge
      ${catkin_LIBRARIES}
    )
  endif()

  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS
    test/test_state_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/state_bridge.test)
  # ################################
  # CMake: State Bridge unit/rostests end
  # ################################
endif()
