cmake_minimum_required(VERSION 3.0.2)
project(expand_padding)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES expand_padding
  CATKIN_DEPENDS
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

add_library(expand_padding src/expand_padding.cpp)
add_dependencies(expand_padding ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# check ros distro >= indigo
execute_process(
  COMMAND python -c "import sys; sys.exit(ord('$ENV{ROS_DISTRO}'[0]) >= ord('melodic'[0]))"
  RESULT_VARIABLE DISTRO_GE_MELODIC
  )

#if(${DISTRO_GE_MELODIC})
  add_definitions(-DUSE_TF_BUFFER)
#endif()

target_link_libraries(expand_padding ${catkin_LIBRARIES})

install(TARGETS expand_padding
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES rotate_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

