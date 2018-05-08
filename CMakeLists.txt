cmake_minimum_required(VERSION 2.8.3)
project(ontologenius)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  pluginlib
  qt_build
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
 add_message_files(
   FILES
   HTTP_headers.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
  FILES
  REST.srv
  standard_service.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ontoloGenius
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS TinyXML
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

##############################
## Declare a C++ library
##############################
add_library(codeDescription_lib
  src/codeDescription/Types/VariablesType.cpp
  src/codeDescription/Types/StringType.cpp
  src/codeDescription/Functions/FunctionDescriptor.cpp
  src/codeDescription/Functions/FunctionContainer.cpp
  src/codeDescription/Functions/OntoFunctions.cpp
  src/codeDescription/TextManipulator.cpp
  src/codeDescription/Code.cpp
  src/codeDescription/Namespace.cpp
  src/codeDescription/Syntax/Operators.cpp
  src/codeDescription/Syntax/IfelseCF.cpp
)
add_dependencies(codeDescription_lib ontologenius_gencpp)

add_library(ontoGraphs_lib
  src/ontoGraphs/Graphs/ClassGraph.cpp
  src/ontoGraphs/Graphs/ObjectPropertyGraph.cpp
  src/ontoGraphs/Graphs/DataPropertyGraph.cpp
  src/ontoGraphs/Graphs/IndividualGraph.cpp
  src/ontoGraphs/Checkers/ClassChecker.cpp
  src/ontoGraphs/Checkers/ObjectPropertyChecker.cpp
  src/ontoGraphs/Checkers/DataPropertyChecker.cpp
  src/ontoGraphs/Checkers/IndividualChecker.cpp
  src/ontoGraphs/Ontology.cpp
  src/ontoGraphs/OntologyReader.cpp
)
add_dependencies(ontoGraphs_lib ontologenius_gencpp)
target_link_libraries(ontoGraphs_lib ${OpenCV_LIBS})
target_link_libraries(ontoGraphs_lib ${TinyXML_LIBRARIES})

add_library(plugin_lib
  src/arguer/plugins/ArguerInterface.cpp
  )
add_dependencies(plugin_lib ontologenius_gencpp)
target_link_libraries(plugin_lib ontoGraphs_lib)

add_library(arguer_lib
  src/arguer/Arguers.cpp
  )
add_dependencies(arguer_lib ontologenius_gencpp)
target_link_libraries(arguer_lib plugin_lib)
target_link_libraries(arguer_lib ${catkin_LIBRARIES})

add_library(ontologenius_lib
  src/Computer.cpp
  src/Parser.cpp
  src/Error.cpp
  src/Compiler.cpp
)
add_dependencies(ontologenius_lib ontologenius_gencpp)
target_link_libraries(ontologenius_lib codeDescription_lib)
target_link_libraries(ontologenius_lib ontoGraphs_lib)

add_library(drawer_lib
  src/ontoGraphs/Drawers/GraphDrawer.cpp
  src/ontoGraphs/Drawers/ClassDrawer.cpp
  src/ontoGraphs/Drawers/ObjectPropertyDrawer.cpp
  src/ontoGraphs/Drawers/DataPropertyDrawer.cpp
  src/ontoGraphs/Drawers/OntologyDrawer.cpp
)
add_dependencies(drawer_lib ontologenius_gencpp)
target_link_libraries(drawer_lib ${catkin_LIBRARIES})
target_link_libraries(drawer_lib ontologenius_lib)

##############################
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
##############################

add_executable(ontoloGenius src/rosInterface.cpp)
add_dependencies(ontoloGenius ontologenius_gencpp)
target_link_libraries(ontoloGenius ${catkin_LIBRARIES})
target_link_libraries(ontoloGenius ontologenius_lib)
target_link_libraries(ontoloGenius arguer_lib)

add_executable(ontoloGeniusExemple src/exemple.cpp )
add_dependencies(ontoloGeniusExemple ontologenius_gencpp)
target_link_libraries(ontoloGeniusExemple drawer_lib)
target_link_libraries(ontoloGeniusExemple ontologenius_lib)

add_executable(ontologenius_tester src/tester.cpp)
add_dependencies(ontologenius_tester ontologenius_gencpp)
target_link_libraries(ontologenius_tester ${catkin_LIBRARIES})
target_link_libraries(ontologenius_tester ontologenius_lib)
target_link_libraries(ontologenius_tester arguer_lib)

## Declare a C++ lib for plugins
add_library(arguer_plugin
  src/arguer/plugins/ArguerNone.cpp
  src/arguer/plugins/ArguerInverseOf.cpp
  src/arguer/plugins/ArguerSymetric.cpp
  src/arguer/plugins/ArguerChain.cpp
  )
add_dependencies(arguer_plugin ontologenius_gencpp)
target_link_libraries(arguer_plugin plugin_lib)

##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}  resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
)

add_definitions( -std=c++11 -fPIC)

##############################################################################
# Sources
##############################################################################

set( QT_SOURCES
    src/ontoloGUI/main.cpp
    src/ontoloGUI/ontologui.cpp
    src/ontoloGUI/DarkStyle.cpp
    src/ontoloGUI/qpushbuttonextended.cpp
    src/ontoloGUI/QCheckBoxExtended.cpp

    include/ontoloGenius/ontoloGUI/ontologui.h
    include/ontoloGenius/ontoloGUI/DarkStyle.h
    include/ontoloGenius/ontoloGUI/qpushbuttonextended.h
    include/ontoloGenius/ontoloGUI/QCheckBoxExtended.h
)
##############################################################################
# Binaries
##############################################################################

add_executable(ontoloGUI ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(ontoloGUI
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS ontoloGUI RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ontoloGenius.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
