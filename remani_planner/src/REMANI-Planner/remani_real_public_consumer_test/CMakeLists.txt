cmake_minimum_required(VERSION 3.0.2)
project(remani_real_public_consumer_test)

find_package(catkin REQUIRED COMPONENTS remani_real)

catkin_package(CATKIN_DEPENDS remani_real)

include_directories(${catkin_INCLUDE_DIRS})

################################
# Verify remani_real exports an include dir that actually contains Eigen/Core.
################################
set(remani_real_eigen_exported FALSE)
foreach(include_dir ${catkin_INCLUDE_DIRS})
  if(EXISTS "${include_dir}/Eigen/Core")
    set(remani_real_eigen_exported TRUE)
  endif()
endforeach()
if(NOT remani_real_eigen_exported)
  message(FATAL_ERROR "remani_real must export Eigen through its catkin Config")
endif()
################################

add_executable(test_public_headers test/test_public_headers.cpp)
target_compile_features(test_public_headers PRIVATE cxx_std_14)
target_link_libraries(test_public_headers ${catkin_LIBRARIES})
