cmake_minimum_required(VERSION 3.0.2)
project(remani_real_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  quadrotor_msgs
  std_msgs
)

add_message_files(FILES PlannerStatus.msg ExecutionState.msg ExecutionResult.msg FrozenCandidate.msg Cr10Status.msg)
add_service_files(FILES ExecuteCandidate.srv)
generate_messages(DEPENDENCIES geometry_msgs nav_msgs quadrotor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs quadrotor_msgs std_msgs)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_execution_state_schema
    test/test_execution_state_schema.cpp)
  if(TARGET test_execution_state_schema)
    target_compile_definitions(test_execution_state_schema PRIVATE
      EXECUTION_STATE_MSG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/msg/ExecutionState.msg")
  endif()
endif()
