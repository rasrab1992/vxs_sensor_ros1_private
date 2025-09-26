
cmake_minimum_required(VERSION 3.5)
project(vxs_sensor_ros1)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  #set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

#Default to C++ 17?
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package (catkin REQUIRED COMPONENTS
   roscpp
   roslib
   rospy
   std_msgs
   cv_bridge
   image_transport
   sensor_msgs
   tf
   pcl_conversions
   message_generation
)

list(APPEND OTHER_LIBS
    ${catkin_LIBRARIES})

# Find OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  list(APPEND EXT_NODE_INCLUDE_DIRS
       ${OpenCV_INCLUDE_DIRS})
  list(APPEND OTHER_LIBS
       ${OpenCV_LIBS})
endif()

#Find VXSSDK
#message( "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${SMAKE_MODULE_PATH};/usr/share/cmake/Modules")
find_package(VXSDK REQUIRED)
if (VXSDK_FOUND)
  list(APPEND EXT_NODE_INCLUDE_DIRS ${VXSDK_INCLUDE_DIR})
  list(APPEND OTHER_LIBS ${VXSDK_LIBRARY})
else()
  message("VXSDK not found!")
endif()

# Generate services in the 'srv' folder
add_service_files(
  FILES
   UpdateObservationWindow.srv
)

catkin_package (
  CATKIN_DEPENDS 
	roscpp 
  roslib
	rospy 
	std_msgs 
	cv_bridge 
	image_transport 
	sensor_msgs
	tf
  pcl_conversions
  message_generation
)

include_directories(${catkin_INCLUDE_DIRS})
set(VXS_NODE_INCLUDE_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/publisher
  ${EXT_NODE_INCLUDE_DIRS})

  catkin_package(
    INCLUDE_DIRS ${VXS_NODE_INCLUDE_DIRS}
  )

set(VXS_NODE_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/publisher/vxs_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/publisher/vxs_node.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/publisher/vxs_node_main.cpp)

add_executable(vxs_node ${VXS_NODE_SOURCES})
target_include_directories(vxs_node PUBLIC ${VXS_NODE_INCLUDE_DIRS})
target_link_libraries(vxs_node ${OTHER_LIBS})
add_dependencies(vxs_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install python script
catkin_install_python(PROGRAMS scripts/vxs_py_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(VXS_SUB_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/subscriber
  ${EXT_NODE_INCLUDE_DIRS}
)

set(VXS_SUB_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subscriber/vxs_subscriber.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subscriber/vxs_subscriber.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subscriber/vxs_subscriber_main.cpp)

add_executable(vxs_cpp_subscriber ${VXS_SUB_SOURCES})
target_include_directories(vxs_cpp_subscriber PUBLIC ${VXS_SUB_INCLUDE_DIRS})
target_link_libraries(vxs_cpp_subscriber ${OTHER_LIBS})
add_dependencies(vxs_cpp_subscriber 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


#install(TARGETS
#  vxs_node
#  vxs_cpp_subscriber
#  DESTINATION lib/${PROJECT_NAME})

