cmake_minimum_required(VERSION 3.0.2)
project(remani_real)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  # ################################
  # CMake: Task4 validation/preview deps begin
  # ################################
  mm_config
  plan_env
  visualization_msgs
  # ################################
  # CMake: Task4 validation/preview deps end
  # ################################
  # ################################
  # CMake: Task5 motion-output deps begin
  # ################################
  geometry_msgs
  trajectory_msgs
  # ################################
  # CMake: Task5 motion-output deps end
  # ################################
  # ################################
  # CMake: Task7 control-plane deps begin
  # ################################
  actionlib
  control_msgs
  std_msgs
  std_srvs
  # ################################
  # CMake: Task7 control-plane deps end
  # ################################
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
  CATKIN_DEPENDS mm_config plan_env visualization_msgs geometry_msgs trajectory_msgs actionlib control_msgs std_msgs std_srvs nav_msgs quadrotor_msgs remani_real_msgs roscpp sensor_msgs tf2 tf2_ros mm_controller
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
  # ################################
  # CMake: Task4 validator/preview sources begin
  # ################################
  src/candidate_validator.cpp
  src/preview_publisher.cpp
  src/mm_config_validation_environment.cpp
  # ################################
  # CMake: Task5 deployment SM begin
  # ################################
  src/deployment_state_machine.cpp
  # ################################
  # CMake: Task5 deployment SM end
  # ################################
  # ################################
  # CMake: Task7 dry-run motion output begin
  # ################################
  src/dry_run_motion_output.cpp
  # ################################
  # CMake: Task7 dry-run motion output end
  # ################################
  # ################################
  # CMake: Phase3 Ranger hardware channel begin
  # ################################
  src/topic_ownership_monitor.cpp
  src/ranger_hardware_channel.cpp
  # ################################
  # CMake: Phase3 Ranger hardware channel end
  # ################################
  # ################################
  # CMake: Phase3 base tracking controller begin
  # ################################
  src/base_tracking_controller.cpp
  # ################################
  # CMake: Phase3 base tracking controller end
  # ################################
  # ################################
  # CMake: Phase4 CR10 arm trajectory + hardware channel begin
  # ################################
  src/arm_trajectory_builder.cpp
  src/cr10_hardware_channel.cpp
  # ################################
  # CMake: Phase4 CR10 arm trajectory + hardware channel end
  # ################################
  # ################################
  # CMake: Phase5 synchronized executor begin
  # ################################
  src/execution_clock.cpp
  src/synchronized_executor.cpp
  src/remaining_candidate.cpp
  src/runtime_safety_monitor.cpp
  src/completion_verifier.cpp
  # ################################
  # CMake: Phase5 synchronized executor end
  # ################################
  # ################################
  # CMake: Task4 validator/preview sources end
  # ################################
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

# ################################
# CMake: remani_real_node dry-run control plane begin
# ################################
add_executable(remani_real_node
  src/remani_real_node.cpp
)
target_compile_features(remani_real_node PRIVATE cxx_std_14)
add_dependencies(remani_real_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(remani_real_node
  remani_real_candidate
  ${catkin_LIBRARIES}
)
# ################################
# CMake: remani_real_node dry-run control plane end
# ################################

install(TARGETS remani_real_candidate remani_real_state_bridge remani_state_bridge_node remani_real_node
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

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  OPTIONAL
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
  # CMake: CandidateValidator gtest begin
  # ################################
  catkin_add_gtest(test_candidate_validator
    test/test_candidate_validator.cpp
  )
  if(TARGET test_candidate_validator)
    target_compile_features(test_candidate_validator PRIVATE cxx_std_14)
    target_link_libraries(test_candidate_validator
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: CandidateValidator gtest end
  # ################################

  # ################################
  # CMake: DeploymentStateMachine gtest begin
  # ################################
  catkin_add_gtest(test_deployment_state_machine
    test/test_deployment_state_machine.cpp
  )
  if(TARGET test_deployment_state_machine)
    target_compile_features(test_deployment_state_machine PRIVATE cxx_std_14)
    target_link_libraries(test_deployment_state_machine
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: DeploymentStateMachine gtest end
  # ################################

  # ################################
  # CMake: DryRunMotionOutput gtest begin
  # ################################
  catkin_add_gtest(test_dry_run_output
    test/test_dry_run_output.cpp
  )
  if(TARGET test_dry_run_output)
    target_compile_features(test_dry_run_output PRIVATE cxx_std_14)
    target_link_libraries(test_dry_run_output
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: DryRunMotionOutput gtest end
  # ################################

  # ################################
  # CMake: Phase3 topic ownership gtest/rostest begin
  # ################################
  catkin_add_gtest(test_topic_ownership
    test/test_topic_ownership.cpp
  )
  if(TARGET test_topic_ownership)
    target_compile_features(test_topic_ownership PRIVATE cxx_std_14)
    target_link_libraries(test_topic_ownership
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: Phase3 topic ownership gtest/rostest end
  # ################################

  # ################################
  # CMake: Phase3 base tracking gtest begin
  # ################################
  catkin_add_gtest(test_base_tracking_controller
    test/test_base_tracking_controller.cpp
  )
  if(TARGET test_base_tracking_controller)
    target_compile_features(test_base_tracking_controller PRIVATE cxx_std_14)
    target_link_libraries(test_base_tracking_controller
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: Phase3 base tracking gtest end
  # ################################

  # ################################
  # CMake: Phase4 arm trajectory builder gtest begin
  # ################################
  catkin_add_gtest(test_arm_trajectory_builder
    test/test_arm_trajectory_builder.cpp
  )
  if(TARGET test_arm_trajectory_builder)
    target_compile_features(test_arm_trajectory_builder PRIVATE cxx_std_14)
    target_link_libraries(test_arm_trajectory_builder
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: Phase4 arm trajectory builder gtest end
  # ################################

  # ################################
  # CMake: Phase5 execution clock / synchronized start gtests begin
  # ################################
  catkin_add_gtest(test_execution_clock
    test/test_execution_clock.cpp
  )
  if(TARGET test_execution_clock)
    target_compile_features(test_execution_clock PRIVATE cxx_std_14)
    target_link_libraries(test_execution_clock
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_synchronized_start
    test/test_synchronized_start.cpp
  )
  if(TARGET test_synchronized_start)
    target_compile_features(test_synchronized_start PRIVATE cxx_std_14)
    target_link_libraries(test_synchronized_start
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: Phase5 execution clock / synchronized start gtests end
  # ################################

  # ################################
  # CMake: Phase5 remaining / pause-resume gtests begin
  # ################################
  catkin_add_gtest(test_remaining_candidate
    test/test_remaining_candidate.cpp
  )
  if(TARGET test_remaining_candidate)
    target_compile_features(test_remaining_candidate PRIVATE cxx_std_14)
    target_link_libraries(test_remaining_candidate
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_pause_resume
    test/test_pause_resume.cpp
  )
  if(TARGET test_pause_resume)
    target_compile_features(test_pause_resume PRIVATE cxx_std_14)
    target_link_libraries(test_pause_resume
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: Phase5 remaining / pause-resume gtests end
  # ################################

  # ################################
  # CMake: Phase5 runtime safety monitor gtest begin
  # ################################
  catkin_add_gtest(test_runtime_safety_monitor
    test/test_runtime_safety_monitor.cpp
  )
  if(TARGET test_runtime_safety_monitor)
    target_compile_features(test_runtime_safety_monitor PRIVATE cxx_std_14)
    target_link_libraries(test_runtime_safety_monitor
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: Phase5 runtime safety monitor gtest end
  # ################################

  # ################################
  # CMake: Phase5 completion verifier gtest begin
  # ################################
  catkin_add_gtest(test_completion_verifier
    test/test_completion_verifier.cpp
  )
  if(TARGET test_completion_verifier)
    target_compile_features(test_completion_verifier PRIVATE cxx_std_14)
    target_link_libraries(test_completion_verifier
      remani_real_candidate
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: Phase5 completion verifier gtest end
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
    test/fake_real_feedback_node.py
    test/fake_ee_goal_marker.py
    test/fake_candidate_planner.py
    test/fake_whole_body_system.py
    test/test_real_control_plane.py
    test/test_real_launch_contract.py
    test/test_ranger_topic_isolation.py
    test/fake_ranger_driver_node.py
    test/test_ranger_only_execution.py
    test/fake_cr10_action_server.py
    test/test_cr10_only_execution.py
    test/test_synchronized_dry_run.py
    test/test_pause_resume_abort.py
    test/test_final_completion.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/state_bridge.test)
  add_rostest(test/real_control_plane.test)
  add_rostest(test/real_launch_contract.test)
  add_rostest(test/ranger_topic_isolation.test)
  add_rostest(test/ranger_only_execution.test)
  add_rostest(test/cr10_only_execution.test)
  add_rostest(test/synchronized_dry_run.test)
  add_rostest(test/pause_resume_abort.test)
  add_rostest(test/final_completion.test)
  # ################################
  # CMake: State Bridge unit/rostests end
  # ################################
endif()
