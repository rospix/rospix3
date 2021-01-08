cmake_minimum_required(VERSION 3.1.2)
project(rospix3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  dynamic_reconfigure
  rad_msgs
  mrs_lib
  )

generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure.cfg
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Rospix3
  CATKIN_DEPENDS roscpp mrs_lib dynamic_reconfigure std_msgs rad_msgs
  DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

execute_process(COMMAND
  dpkg-architecture
  -qDEB_HOST_ARCH
  OUTPUT_VARIABLE
  CMAKE_DEB_HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

if(${CMAKE_DEB_HOST_ARCH} MATCHES "armhf")
  MESSAGE(FATAL_ERROR "Mising MpcTrackerSolver.so for armhf")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "i386")
  MESSAGE(FATAL_ERROR "Mising MpcTrackerSolver.so for i386")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "x64")
  set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib/x64)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "amd64")
  set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib/x64)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "arm64")
  set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib/arm64)
else()
  MESSAGE(FATAL_ERROR ${CMAKE_DEB_HOST_ARCH})
  MESSAGE(FATAL_ERROR "external libraries has not been selected, check CMakeLists.txt.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl -L${LIB_DIR}")

add_library(Rospix3 src/rospix3.cpp)

add_dependencies(Rospix3
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(Rospix3
  ${catkin_LIBRARIES}
  ${LIB_DIR}/libpxproc.so
  # ${LIB_DIR}/pxcore.so
  )
