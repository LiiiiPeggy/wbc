cmake_minimum_required(VERSION 3.0.2)
project(remani_real)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mm_controller
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES remani_real_candidate
  CATKIN_DEPENDS roscpp mm_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../mm_controller/include
)

add_library(remani_real_candidate
  src/candidate_trajectory.cpp
)
target_compile_features(remani_real_candidate PUBLIC cxx_std_14)
target_link_libraries(remani_real_candidate
  ${catkin_LIBRARIES}
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
endif()
