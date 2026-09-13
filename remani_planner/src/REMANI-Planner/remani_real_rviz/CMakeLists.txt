cmake_minimum_required(VERSION 3.0.2)
project(remani_real_rviz)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  remani_real_msgs
  roscpp
  rviz
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES remani_real_rviz
  CATKIN_DEPENDS pluginlib remani_real_msgs roscpp rviz std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ################################
# CMake: Qt/RViz panel library begin
# ################################
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_FILES src/remani_real_panel.hpp)
else()
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  qt5_wrap_cpp(MOC_FILES src/remani_real_panel.hpp)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(remani_real_rviz
  src/remani_real_panel.cpp
  ${MOC_FILES}
)
target_compile_features(remani_real_rviz PUBLIC cxx_std_14)
add_dependencies(remani_real_rviz ${catkin_EXPORTED_TARGETS})
target_link_libraries(remani_real_rviz
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# ################################
# CMake: Qt/RViz panel library end
# ################################

install(TARGETS remani_real_rviz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ################################
# CMake: install real RViz config begin
# ################################
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
# ################################
# CMake: install real RViz config end
# ################################

if(CATKIN_ENABLE_TESTING)
  # ################################
  # CMake: PanelViewModel gtest begin
  # ################################
  catkin_add_gtest(test_panel_view_model
    test/test_panel_view_model.cpp
  )
  if(TARGET test_panel_view_model)
    target_compile_features(test_panel_view_model PRIVATE cxx_std_14)
    target_link_libraries(test_panel_view_model
      ${catkin_LIBRARIES}
    )
  endif()
  # ################################
  # CMake: PanelViewModel gtest end
  # ################################
endif()
