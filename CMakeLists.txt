cmake_minimum_required(VERSION 3.0.0)
project(sas_robot_kinematics)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Werror=return-type)

FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pybind11_catkin
    sas_conversions
    sas_common
    geometry_msgs
    )

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
    LIBRARIES sas_robot_kinematics
    CATKIN_DEPENDS sas_conversions roscpp rospy geometry_msgs sas_common pybind11_catkin
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/sas_robot_kinematics_provider.cpp
   src/sas_robot_kinematics_interface.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(sas_robot_kinematics
 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(sas_robot_kinematics
    dqrobotics
    ${catkin_LIBRARIES}
    )
    
#####################
## PYTHON BINDINGS ##
#####################    
pybind_add_module(_sas_robot_kinematics SHARED src/sas_robot_kinematics_py.cpp src/sas_robot_kinematics_interface.cpp src/sas_robot_kinematics_provider.cpp)
target_compile_definitions(_sas_robot_kinematics PRIVATE IS_SAS_PYTHON_BUILD)
# https://github.com/pybind/pybind11/issues/387
target_link_libraries(_sas_robot_kinematics PRIVATE ${catkin_LIBRARIES} -ldqrobotics)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/sas_robot_kinematics_interface_example.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

install(TARGETS _sas_robot_kinematics
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

catkin_python_setup()
