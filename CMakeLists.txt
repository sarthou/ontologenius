cmake_minimum_required(VERSION 2.8.3)
project(ontologenius)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

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
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)


################################################
## Declare ROS messages, services and actions ##
################################################

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
  OntologeniusService.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

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
  LIBRARIES ontoloGenius_lib
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
##############################
## CORE
##############################
add_library(ontologenius_ontoGraphs_lib
  src/core/ontoGraphs/Branchs/ClassBranch.cpp
  src/core/ontoGraphs/Branchs/ObjectPropertyBranch.cpp
  src/core/ontoGraphs/Branchs/DataPropertyBranch.cpp
  src/core/ontoGraphs/Branchs/IndividualBranch.cpp
  src/core/ontoGraphs/Branchs/ValuedNode.cpp
  src/core/ontoGraphs/Graphs/ClassGraph.cpp
  src/core/ontoGraphs/Graphs/ObjectPropertyGraph.cpp
  src/core/ontoGraphs/Graphs/DataPropertyGraph.cpp
  src/core/ontoGraphs/Graphs/IndividualGraph.cpp
  src/core/ontoGraphs/Checkers/ClassChecker.cpp
  src/core/ontoGraphs/Checkers/ObjectPropertyChecker.cpp
  src/core/ontoGraphs/Checkers/DataPropertyChecker.cpp
  src/core/ontoGraphs/Checkers/IndividualChecker.cpp
  src/core/ontoGraphs/Ontology.cpp
  src/core/ontologyIO/OntologyReader.cpp
  src/core/ontologyIO/OntologyWriter.cpp
  src/core/ontologyIO/writers/NodeWriter.cpp
  src/core/ontologyIO/writers/ClassWriter.cpp
  src/core/ontologyIO/writers/ObjectPropertiesWriter.cpp
  src/core/ontologyIO/writers/DataPropertiesWriter.cpp
  src/core/ontologyIO/writers/IndividualWriter.cpp
)
add_dependencies(ontologenius_ontoGraphs_lib ontologenius_gencpp)
target_link_libraries(ontologenius_ontoGraphs_lib ${OpenCV_LIBS})
target_link_libraries(ontologenius_ontoGraphs_lib ${TinyXML_LIBRARIES})

add_library(ontologenius_core_lib
  src/core/feeder/Feeder.cpp
  src/core/feeder/FeedStorage.cpp
  src/core/arguer/plugins/ArguerInterface.cpp
  src/core/arguer/Arguers.cpp
  src/core/Computer.cpp
  )
add_dependencies(ontologenius_core_lib ontologenius_gencpp)
target_link_libraries(ontologenius_core_lib ${catkin_LIBRARIES})
target_link_libraries(ontologenius_core_lib ontologenius_ontoGraphs_lib)

##############################
## DRAWER
##############################

add_library(ontologenius_drawer_lib
  src/graphical/Drawers/GraphDrawer.cpp
  src/graphical/Drawers/ClassDrawer.cpp
  src/graphical/Drawers/ObjectPropertyDrawer.cpp
  src/graphical/Drawers/DataPropertyDrawer.cpp
  src/graphical/Drawers/OntologyDrawer.cpp
)
add_dependencies(ontologenius_drawer_lib ontologenius_gencpp)
target_link_libraries(ontologenius_drawer_lib ${catkin_LIBRARIES})
target_link_libraries(ontologenius_core_lib ontologenius_ontoGraphs_lib)

##############################
## INTERPRETER
##############################

add_library(ontologenius_codeDescription_lib
  src/interpreter/codeDescription/Types/VariablesType.cpp
  src/interpreter/codeDescription/Types/StringType.cpp
  src/interpreter/codeDescription/Functions/FunctionDescriptor.cpp
  src/interpreter/codeDescription/Functions/FunctionContainer.cpp
  src/interpreter/codeDescription/Functions/OntoFunctions.cpp
  src/interpreter/codeDescription/Functions/PropertyFunctions.cpp
  src/interpreter/codeDescription/TextManipulator.cpp
  src/interpreter/codeDescription/Code.cpp
  src/interpreter/codeDescription/Namespace.cpp
  src/interpreter/codeDescription/Syntax/Operators.cpp
  src/interpreter/codeDescription/Syntax/IfelseCF.cpp
  src/interpreter/fileManagers/FileManager.cpp
  src/interpreter/fileManagers/FileWriter.cpp
)
add_dependencies(ontologenius_codeDescription_lib ontologenius_gencpp)

add_library(ontologenius_interpreter_lib
  src/interpreter/Parser.cpp
  src/interpreter/Error.cpp
  src/interpreter/Compiler.cpp
)
add_dependencies(ontologenius_interpreter_lib ontologenius_gencpp)
target_link_libraries(ontologenius_interpreter_lib ontologenius_codeDescription_lib)
target_link_libraries(ontologenius_interpreter_lib ontologenius_ontoGraphs_lib)

add_library(ontoloGenius_lib
  src/utility/clients/ontologyClients/IndividualClient.cpp
  src/utility/clients/ontologyClients/ObjectPropertyClient.cpp
  src/utility/clients/ontologyClients/DataPropertyClient.cpp
  src/utility/clients/ontologyClients/ClassClient.cpp
  src/utility/clients/ontologyClients/OntologyClient.cpp
  src/utility/clients/ArguerClient.cpp
  src/utility/clients/ActionClient.cpp
  src/utility/clients/ClientBase.cpp
  src/utility/FeederPublisher.cpp
  src/utility/OntologyManipulator.cpp
)
add_dependencies(ontoloGenius_lib ontologenius_gencpp)
target_link_libraries(ontoloGenius_lib ontologenius_interpreter_lib)
target_link_libraries(ontoloGenius_lib ontologenius_core_lib)

##############################
## PLUGINS
##############################
add_library(ontologenius_arguer_plugin
  src/core/arguer/plugins/ArguerNone.cpp
  src/core/arguer/plugins/ArguerInverseOf.cpp
  src/core/arguer/plugins/ArguerSymetric.cpp
  src/core/arguer/plugins/ArguerChain.cpp
  src/core/arguer/plugins/ArguerDictionary.cpp
  src/core/arguer/plugins/ArguerGeneralize.cpp
  )
add_dependencies(ontologenius_arguer_plugin ontologenius_gencpp)
target_link_libraries(ontologenius_arguer_plugin ontologenius_core_lib)

##############################
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
##############################

add_executable(ontoloGenius src/rosInterface.cpp)
add_dependencies(ontoloGenius ontologenius_gencpp)
target_link_libraries(ontoloGenius ontologenius_interpreter_lib)
target_link_libraries(ontoloGenius ontologenius_core_lib)

add_executable(ontoloGenius_exemple src/exemple.cpp )
add_dependencies(ontoloGenius_exemple ontologenius_gencpp)
target_link_libraries(ontoloGenius_exemple ontologenius_drawer_lib)
target_link_libraries(ontoloGenius_exemple ontologenius_core_lib)

add_executable(ontoloGenius_tester src/tests/tester.cpp)
add_dependencies(ontoloGenius_tester ontologenius_gencpp)
target_link_libraries(ontoloGenius_tester ontologenius_core_lib)

add_executable(ontoloGenius_container_tester src/tests/container_test.cpp)
add_dependencies(ontoloGenius_container_tester ontologenius_gencpp)
target_link_libraries(ontoloGenius_container_tester ontologenius_core_lib)

add_executable(ontoloGenius_services_tester src/tests/service_tester.cpp)
add_dependencies(ontoloGenius_services_tester ontologenius_gencpp)
target_link_libraries(ontoloGenius_services_tester ontologenius_core_lib)

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
    src/graphical/ontoloGUI/main.cpp
    src/graphical/ontoloGUI/ontologui.cpp
    src/graphical/ontoloGUI/DarkStyle.cpp
    src/graphical/ontoloGUI/qpushbuttonextended.cpp
    src/graphical/ontoloGUI/QCheckBoxExtended.cpp

    include/ontoloGenius/graphical/ontoloGUI/ontologui.h
    include/ontoloGenius/graphical/ontoloGUI/DarkStyle.h
    include/ontoloGenius/graphical/ontoloGUI/qpushbuttonextended.h
    include/ontoloGenius/graphical/ontoloGUI/QCheckBoxExtended.h
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

##############################
## Install
##############################

install(TARGETS
  ontoloGenius_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

##############################
## Tests
##############################

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(ontoloGenius_library_tester test/library.test src/tests/library.cpp)
  target_link_libraries(ontoloGenius_library_tester ontoloGenius_lib)

  add_rostest_gtest(ontoloGenius_requests_tester test/requests.test src/tests/requests.cpp)
  target_link_libraries(ontoloGenius_requests_tester ontoloGenius_lib)

  add_rostest_gtest(ontoloGenius_global_tester test/globalFeatures.test src/tests/globalFeatures.cpp)
  target_link_libraries(ontoloGenius_global_tester ontoloGenius_lib)

  add_rostest_gtest(ontoloGenius_propertiesInheritage_tester test/propertiesInheritage.test src/tests/propertiesInheritage.cpp)
  target_link_libraries(ontoloGenius_propertiesInheritage_tester ontoloGenius_lib)

  add_rostest_gtest(ontoloGenius_dynamic_tester test/dynamic.test src/tests/dynamic.cpp)
  target_link_libraries(ontoloGenius_dynamic_tester ontoloGenius_lib)
endif()
