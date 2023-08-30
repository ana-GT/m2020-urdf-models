cmake_minimum_required(VERSION 3.0.2)
project(m2020_urdf_models)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  xacro
  gazebo_ros
  geometry_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mhs_gazebo_plugin
  CATKIN_DEPENDS roscpp xacro gazebo_ros geometry_msgs
  DEPENDS 
)

include_directories(
include
${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}


link_directories(${catkin_LIBRARY_DIRS}))


add_library(mhs_gazebo_plugin SHARED src/mhs_gazebo_plugin.cpp)
target_link_libraries(mhs_gazebo_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(mhs_gazebo_plugin ${catkin_EXPORTED_TARGETS})


install(TARGETS
  mhs_gazebo_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)



