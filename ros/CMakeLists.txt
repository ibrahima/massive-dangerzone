cmake_minimum_required(VERSION 2.8.3)
project(moveit_sqp_planner_ros_plugin)

find_package(Boost REQUIRED system filesystem date_time thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(bulletsim REQUIRED)
message("BULLETSIM include dirs")
message(${bulletsim_INCLUDE_DIRS})
find_package(catkin COMPONENTS
  moveit_ros_planning
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  REQUIRED)

find_package(OpenThreads REQUIRED)
find_package(osg REQUIRED)
find_package(osgDB REQUIRED)
find_package(osgGA REQUIRED)
find_package(osgText REQUIRED)
find_package(osgUtil REQUIRED)
find_package(osgViewer REQUIRED)
# This is better than the above, I think.
find_package(OpenSceneGraph REQUIRED osgDB osgGA osgText osgUtil osgViewer)

set(OSG_LIBS ${OPENTHREADS_LIBRARY} ${OSG_LIBRARY} ${OSGDB_LIBRARY} ${OSGGA_LIBRARY} ${OSGTEXT_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGVIEWER_LIBRARY})
set(OSG_INCLUDE_DIRS ${OPENTHREADS_LIBRARY_INCLUDE_DIR} ${OSG_LIBRARY_INCLUDE_DIR} ${OSGDB_LIBRARY_INCLUDE_DIR} ${OSGGA_LIBRARY_INCLUDE_DIR} ${OSGTEXT_LIBRARY_INCLUDE_DIR} ${OSGUTIL_LIBRARY_INCLUDE_DIR} ${OSGVIEWER_LIBRARY_INCLUDE_DIR})

set(OSGWORKS_DIR ${BULLETSIM_SOURCE_DIR}/lib/osgWorks-2.0)
set(OSGBULLET_DIR ${BULLETSIM_SOURCE_DIR}/lib/osgBullet-2.0)
set(JSON_DIR ${BULLETSIM_SOURCE_DIR}/lib/json)
set(JSON_INCLUDE_DIR ${JSON_DIR}/include)
set(TETGEN_DIR ${BULLETSIM_SOURCE_DIR}/lib/tetgen-1.4.3)
set(LOG4CPLUS_DIR ${BULLETSIM_SOURCE_DIR}/lib/log4cplus-1.1.0-rc3)
set(LOG4CPLUS_INCLUDE_DIRS ${LOG4CPLUS_DIR}/include ${CMAKE_BINARY_DIR}/include)
set(LOG4CPLUS_LIBRARY "log4cplus")

include_directories(${OSG_INCLUDE_DIRS})
link_directories(${OSG_LIBS})
include_directories(${OSGBULLET_DIR})

include_directories(${LOG4CPLUS_INCLUDE_DIRS})
link_directories(${LOG4CPLUS_DIR})

message("Bulletsim dependencies:")
message("osg: ${osg_INCLUDE_DIRS}")
message("OSG-Found: ${osg_Found}")
message("OSGBullet: ${OSGBULLET_DIR}")
message("Bullet_dir: ${BULLET_DIR}")
message("BulletSim libraries: ${bulletsim_LIBRARY_DIRS}")
message("CMAKE_FIND_LIBRARY_PREFIXES: ${CMAKE_FIND_LIBRARY_PREFIXES}")

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${bulletsim_INCLUDE_DIRS})
link_directories(${bulletsim_LIBRARY_DIRS})

# Trying to get log4cplus to work

add_subdirectory(${LOG4CPLUS_DIR} ${CATKIN_DEVEL_PREFIX})

# TODO: The includes should be reorganized into their own dir in bulletsim
include_directories("${BULLET_DIR}/src")
# TODO: Allow configuring gurobi path
include_directories("/opt/gurobi501/linux64/include")
find_package(OpenRAVE 0.8 REQUIRED)
include_directories(${OpenRAVE_INCLUDE_DIRS})
message("${OpenRAVE_INCLUDE_DIRS}")
message("${OpenRAVE_FOUND}")
catkin_package(
  LIBRARIES
    moveit_sqpp_interface_ros
  INCLUDE_DIRS
    #cfg/cpp
    sqpp_interface_ros/include
  DEPENDS 
    moveit_ros_planning
    )

#generate_dynamic_reconfigure_options("ompl_interface_ros/cfg/OMPLParamsDR.cfg")

#include_directories(cfg/cpp)
include_directories(sqpp_interface_ros/include)
add_subdirectory(sqpp_interface_ros)
