cmake_minimum_required(VERSION 3.0.2)
project(remani_real_public_consumer_test)

find_package(catkin REQUIRED COMPONENTS remani_real)

catkin_package(CATKIN_DEPENDS remani_real)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_public_headers test/test_public_headers.cpp)
target_compile_features(test_public_headers PRIVATE cxx_std_14)
target_link_libraries(test_public_headers ${catkin_LIBRARIES})
