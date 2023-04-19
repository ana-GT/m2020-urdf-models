cmake_minimum_required(VERSION 3.0.2)
project(m2020_urdf_models)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  xacro
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES attach_model_plugin
  CATKIN_DEPENDS roscpp xacro
  DEPENDS 
)

#include_directories(
# include
# ${catkin_INCLUDE_DIRS}
#)

#link_directories(${catkin_LIBRARY_DIRS})


#install(TARGETS
#  gazebo_grasp_ui attach_model_plugin move_model_plugin
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


