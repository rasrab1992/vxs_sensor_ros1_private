
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
   rospy
   std_msgs
   cv_bridge
   image_transport
   sensor_msgs
   tf
   message_generation
   nodelet
   orb_slam2
   poser_msgs
   pcl_conversions
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

catkin_package (
  CATKIN_DEPENDS 
	roscpp 
	rospy 
	std_msgs 
	cv_bridge 
	image_transport 
	sensor_msgs
	tf
  message_runtime
  nodelet
  orb_slam2
  pcl_conversions
)


list(APPEND VXS_NODE_INCLUDE_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EXT_NODE_INCLUDE_DIRS})

include_directories(${VXS_NODE_INCLUDE_DIRS})
set(VXS_NODE_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/publisher/vxs_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/publisher/vxs_node.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/publisher/vxs_node_main.cpp)

add_executable(vxs_node ${VXS_NODE_SOURCES})
target_link_libraries(vxs_node ${OTHER_LIBS})

SET(VXS_CPP_SUB_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subscriber/vxs_subscriber.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subscriber/vxs_subscriber.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subscriber/vxs_subscriber_main.cpp)

add_executable(vxs_cpp_subscriber ${VXS_CPP_SUB_SOURCES})
target_link_libraries(vxs_cpp_subscriber ${OTHER_LIBS})


#install(TARGETS
#  vxs_node
#  vxs_cpp_subscriber
#  DESTINATION lib/${PROJECT_NAME})

