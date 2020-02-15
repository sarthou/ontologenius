cmake_minimum_required(VERSION 3.5)
project(ontologenius)

include(cmake/ClangTidy.cmake)

function(add_onto_library TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()
    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()
    add_library(${TARGET} ${ARGN})
    add_dependencies(${TARGET} ontologenius_gencpp ${catkin_EXPORTED_TARGETS})
    target_include_directories(${TARGET} PUBLIC include PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PRIVATE ${catkin_LIBRARIES})
    set_target_properties(${TARGET} PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
endfunction(add_onto_library)

function(add_onto_executable TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()
    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()
    add_executable(${TARGET} ${ARGN})
    #add_dependencies(${TARGET} ${catkin_EXPORTED_TARGETS})
    target_include_directories(${TARGET} PUBLIC include PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PRIVATE ${catkin_LIBRARIES})
    set_target_properties(${TARGET} PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
endfunction(add_onto_executable)


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
  INCLUDE_DIRS include include include/ontologenius/API
  LIBRARIES ontologenius_lib
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS TinyXML
)

###########
## Build ##
###########

##############################
## Declare a C++ library
##############################
##############################
## CORE
##############################

add_onto_library(ontologenius_ontoGraphs_lib
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
target_include_directories(ontologenius_ontoGraphs_lib
 PUBLIC
  ${TinyXML_INCLUDE_DIRS}
)
target_link_libraries(ontologenius_ontoGraphs_lib
 PUBLIC
  ${TinyXML_LIBRARIES}
)

add_onto_library(ontologenius_core_lib
  src/core/feeder/Feeder.cpp
  src/core/feeder/FeedStorage.cpp
  src/core/reasoner/plugins/ReasonerInterface.cpp
  src/core/reasoner/Reasoners.cpp
  src/core/reasoner/ConfigReader.cpp
  src/core/Computer.cpp
)
target_link_libraries(ontologenius_core_lib
 PUBLIC
  ontologenius_ontoGraphs_lib
)

##############################
## DRAWER
##############################

add_onto_library(ontologenius_drawer_lib
  src/graphical/Drawers/GraphDrawer.cpp
  src/graphical/Drawers/ClassDrawer.cpp
  src/graphical/Drawers/ObjectPropertyDrawer.cpp
  src/graphical/Drawers/DataPropertyDrawer.cpp
  src/graphical/Drawers/OntologyDrawer.cpp
)
target_include_directories(ontologenius_drawer_lib
PUBLIC
 ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ontologenius_drawer_lib
 PUBLIC
  ontologenius_ontoGraphs_lib
  ${OpenCV_LIBS}
)

##############################
## INTERPRETER
##############################

add_onto_library(ontologenius_codeDescription_lib
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

add_onto_library(ontologenius_interpreter_lib
  src/interpreter/Parser.cpp
  src/interpreter/Error.cpp
  src/interpreter/Compiler.cpp
)
target_link_libraries(ontologenius_interpreter_lib
 PUBLIC
  ontologenius_codeDescription_lib
  ontologenius_ontoGraphs_lib
)

add_onto_library(ontologenius_lib
  src/API/ontologenius/clients/ontologyClients/IndividualClient.cpp
  src/API/ontologenius/clients/ontologyClients/ObjectPropertyClient.cpp
  src/API/ontologenius/clients/ontologyClients/DataPropertyClient.cpp
  src/API/ontologenius/clients/ontologyClients/ClassClient.cpp
  src/API/ontologenius/clients/ontologyClients/OntologyClient.cpp
  src/API/ontologenius/clients/ReasonerClient.cpp
  src/API/ontologenius/clients/ActionClient.cpp
  src/API/ontologenius/clients/ManagerClient.cpp
  src/API/ontologenius/clients/ClientBase.cpp
  src/API/ontologenius/FeederPublisher.cpp
  src/API/ontologenius/OntologyManipulator.cpp
  src/API/ontologenius/OntologiesManipulator.cpp
)
target_link_libraries(ontologenius_lib
 PUBLIC
  ontologenius_interpreter_lib
  ontologenius_core_lib
)

##############################
## PLUGINS
##############################
add_onto_library(ontologenius_reasoner_plugin
  src/core/reasoner/plugins/ReasonerNone.cpp
  src/core/reasoner/plugins/ReasonerInverseOf.cpp
  src/core/reasoner/plugins/ReasonerSymetric.cpp
  src/core/reasoner/plugins/ReasonerChain.cpp
  src/core/reasoner/plugins/ReasonerDictionary.cpp
  src/core/reasoner/plugins/ReasonerGeneralize.cpp
  src/core/reasoner/plugins/ReasonerRangeDomain.cpp
)
target_link_libraries(ontologenius_reasoner_plugin
 PUBLIC
  ontologenius_core_lib
)

##############################
## INTERFACE
##############################

add_onto_library(ontologenius_interface
  src/RosInterface.cpp
  src/core/ontologyOperators/differenceFinder.cpp
)
target_link_libraries(ontologenius_interface
 PUBLIC
  ontologenius_core_lib
)

##############################
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
##############################

add_onto_executable(ontologenius src/nodes/ontologenius_single.cpp)
target_link_libraries(ontologenius PRIVATE ontologenius_interpreter_lib ontologenius_interface)

add_onto_executable(ontologenius_multi src/nodes/ontologenius_multi.cpp)
target_link_libraries(ontologenius_multi PRIVATE ontologenius_interface)

add_onto_executable(ontologenius_exemple src/exemple.cpp )
target_link_libraries(ontologenius_exemple PRIVATE ontologenius_drawer_lib ontologenius_core_lib)

add_onto_executable(ontologenius_tester src/tests/tester.cpp)
target_link_libraries(ontologenius_tester PRIVATE ontologenius_core_lib)

add_onto_executable(ontologenius_container_tester src/tests/container_test.cpp)
target_link_libraries(ontologenius_container_tester PRIVATE ontologenius_core_lib)

add_onto_executable(ontologenius_find_tester src/tests/getNameVsFind.cpp)
target_link_libraries(ontologenius_find_tester PRIVATE ontologenius_lib ontologenius_core_lib)

add_onto_executable(ontologenius_ORO src/tests/ORO.cpp)
target_link_libraries(ontologenius_ORO PRIVATE ontologenius_lib ontologenius_core_lib)

add_onto_executable(ontologenius_services_tester src/tests/service_tester.cpp)
target_link_libraries(ontologenius_services_tester PRIVATE ontologenius_core_lib)

add_onto_executable(ontologenius_config_tester src/tests/config_reader_test.cpp)
target_link_libraries(ontologenius_config_tester PRIVATE ontologenius_core_lib)

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
    src/graphical/ontoloGUI/QLineEditExtended.cpp

    include/ontologenius/graphical/ontoloGUI/ontologui.h
    include/ontologenius/graphical/ontoloGUI/DarkStyle.h
    include/ontologenius/graphical/ontoloGUI/qpushbuttonextended.h
    include/ontologenius/graphical/ontoloGUI/QCheckBoxExtended.h
    include/ontologenius/graphical/ontoloGUI/QLineEditExtended.h
)
##############################################################################
# Binaries
##############################################################################

add_executable(ontoloGUI ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_include_directories(ontoloGUI
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)
add_dependencies(ontoloGUI ontologenius_gencpp)
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
  ontologenius_lib
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

  add_rostest_gtest(ontologenius_library_tester test/library.test src/tests/library.cpp)
  target_include_directories(ontologenius_library_tester PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_library_tester ontologenius_lib ${catkin_LIBRARIES})

  add_rostest_gtest(ontologenius_requests_tester test/requests.test src/tests/requests.cpp)
  target_include_directories(ontologenius_requests_tester PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_requests_tester ontologenius_lib ${catkin_LIBRARIES})

  add_rostest_gtest(ontologenius_global_tester test/globalFeatures.test src/tests/globalFeatures.cpp)
  target_include_directories(ontologenius_global_tester PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_global_tester ontologenius_lib ${catkin_LIBRARIES})

  add_rostest_gtest(ontologenius_propertiesInheritage_tester test/propertiesInheritage.test src/tests/propertiesInheritage.cpp)
  target_include_directories(ontologenius_propertiesInheritage_tester PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_propertiesInheritage_tester ontologenius_lib ${catkin_LIBRARIES})

  add_rostest_gtest(ontologenius_dynamic_tester test/dynamic.test src/tests/dynamic.cpp)
  target_include_directories(ontologenius_dynamic_tester PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_dynamic_tester ontologenius_lib ${catkin_LIBRARIES})

  add_rostest_gtest(ontologenius_multi_tester test/multi.test src/tests/multi.cpp)
  target_include_directories(ontologenius_multi_tester PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_multi_tester ontologenius_lib ${catkin_LIBRARIES})

  add_rostest_gtest(ontologenius_cpy_test test/cpy.test src/tests/copy_test.cpp)
  set_target_properties(ontologenius_cpy_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
  target_include_directories(ontologenius_cpy_test PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ontologenius_cpy_test ontologenius_lib ${catkin_LIBRARIES})
endif()
