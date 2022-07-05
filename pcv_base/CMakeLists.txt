cmake_minimum_required(VERSION 3.6)
project(pcv_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED NO_MODULE)

set(PCV_BASE_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(PCV_BASE_EXT_DIR ${PROJECT_SOURCE_DIR}/external)
#set(PCV_BASE_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

if(NOT (EXISTS ${PCV_BASE_EXT_DIR}))
  make_directory(${PCV_BASE_EXT_DIR})
endif()

# Find dependencies
function(download_package PACKAGE_NAME)
    configure_file(${PCV_BASE_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt)
    # since ctrl_utils "hard-coded" the use of Unix Makefiles, we replace the ${CMAKE_GENERATOR} with "Unix Makefiles" here as a fix.
    execute_process(COMMAND ${CMAKE_COMMAND} -G "Unix Makefiles" .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(ctrl_utils 1.0.0 QUIET)
#if(NOT ctrl_utils_FOUND)
#    download_package(ctrl_utils)
#    find_package(ctrl_utils 1.0.0 REQUIRED)
#endif()

find_package(realsense2 HINTS /usr/local/include/librealsense2)
set (LIB_REALSENSE_LIBRARIES 
		${realsense2_LIBRARY})
		
#option to use OTG
option(USE_OTG "use the Reflexxes Library for trajectory generation" ON)
if(USE_OTG)
  set (OTG_SOURCE_DIR ${PCV_BASE_EXT_DIR}/ReflexxesTypeII)
  set (OTG_LIBRARY ${OTG_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so)
  #find_package(ReflexxesTypeII QUIET)
	#if(NOT ReflexxesTypeII_FOUND)
  # ReflexxesTypeII is not CMake-based. Use a find if directory exists strategy instead.
  if(NOT (EXISTS ${OTG_SOURCE_DIR}))
    	download_package(ReflexxesTypeII)
    	# use a find if directory exists strategy instead.
    	#find_package(ReflexxesTypeII REQUIRED)
	endif()
  # if binary found -- if not make the downloaded lib.
	if(NOT (EXISTS ${OTG_LIBRARY}))
    # first create directory structure: under Linux$ mkdir -p x64/release/lib/static
    #												                         mkdir -p x64/release/obj/static
    #											                           mkdir -p x64/release/bin/static
    # under Linux$ make all_release_x64
    make_directory(${OTG_SOURCE_DIR}/Linux/x64/release/lib/shared/)
    make_directory(${OTG_SOURCE_DIR}/Linux/x64/release/obj/shared/)
    make_directory(${OTG_SOURCE_DIR}/Linux/x64/release/bin/shared/)
    execute_process(
      COMMAND make all_release_shared_x64
      WORKING_DIRECTORY ${OTG_SOURCE_DIR}/Linux
    )
  endif()
	set (PROJECT_DEFINITIONS -DUSING_OTG)
	set (OTG_INCLUDE_DIRS ${OTG_SOURCE_DIR}/include)
	set (OTG_SOURCE
			${PROJECT_SOURCE_DIR}/src/OTG.cc)
	add_definitions(-DUSING_OTG)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  omniveyor_common
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    std_msgs
#    nav_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pcv_base
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs omniveyor_common tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OTG_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/external
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pcv_base.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/pcv_base_node.cc
  src/caster.cc
  src/vehicle.cc
  src/motor.c
  src/OTG.cc
  #src/pose_t265.cc
  src/CAN_utils.c
  src/CO_message.c
  src/RT_utils.c
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OTG_LIBRARY}
  ${LIB_REALSENSE_LIBRARIES}
  rt
  pthread
  Eigen3::Eigen
  #ctrl_utils::ctrl_utils
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcv_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
