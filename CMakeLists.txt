cmake_minimum_required(VERSION 3.8)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(ontologenius)

option(WITH_CLANG_TIDY "Enable clang-tidy static analysis during build" OFF)

if(WITH_CLANG_TIDY)
  include(cmake/ClangTidy.cmake)
endif()

include(cmake/Sanitizers.cmake)

function(add_onto_library TARGET)
  if(NOT TARGET)
    message(FATAL_ERROR "Expected the target name as first argument")
  endif()

  if(NOT ARGN)
    message(FATAL_ERROR "Expected source file list after target name")
  endif()

  add_library(${TARGET} ${ARGN})
  target_include_directories(${TARGET} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  set_target_properties(${TARGET} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
  target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
  target_compile_options(${TARGET} PRIVATE $<$<CXX_COMPILER_ID:Clang>:-frelaxed-template-template-args>)
  target_enable_sanitizers(${TARGET})
endfunction(add_onto_library)

# ###############################################
# #          Find macros and libraries         ##
# ###############################################
find_package(PkgConfig REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
find_package(OpenCV REQUIRED)

pkg_check_modules(TINYXML2 REQUIRED tinyxml2)
pkg_search_module(CURL REQUIRED libcurl IMPORTED_TARGET GLOBAL)

if($ENV{ROS_VERSION} STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    std_msgs
    genmsg
    message_generation
  )

  # Install python scripts using distutils
  catkin_python_setup()

elseif($ENV{ROS_VERSION} STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(std_msgs REQUIRED)

  # install(PROGRAMS
  # ontopy/${PROJECT_NAME}
  # DESTINATION lib)
  ament_python_install_package(ontopy)
endif()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################
if($ENV{ROS_VERSION} STREQUAL "1")
  # # Generate messages in the 'msg' folder
  add_message_files(
    FILES
    OntologeniusTimestamp.msg
    OntologeniusStampedString.msg
    OntologeniusSparqlResponse.msg
    OntologeniusSparqlIndexResponse.msg
    OntologeniusExplanation.msg
    OntologeniusSubscriptionAnswer.msg
  )

  # # Generate services in the 'srv' folder
  add_service_files(
    FILES
    OntologeniusConversion.srv
    OntologeniusService.srv
    OntologeniusIndexService.srv
    OntologeniusSparqlService.srv
    OntologeniusSparqlIndexService.srv
    OntologeniusSubscription.srv
    OntologeniusUnsubscription.srv
  )

  # # Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
elseif($ENV{ROS_VERSION} STREQUAL "2")
  rosidl_generate_interfaces(ontologenius
    "msg/OntologeniusTimestamp.msg"
    "msg/OntologeniusExplanation.msg"
    "msg/OntologeniusSparqlIndexResponse.msg"
    "msg/OntologeniusSparqlResponse.msg"
    "msg/OntologeniusStampedString.msg"
    "msg/OntologeniusSubscriptionAnswer.msg"
    "srv/OntologeniusConversion.srv"
    "srv/OntologeniusIndexService.srv"
    "srv/OntologeniusService.srv"
    "srv/OntologeniusSparqlIndexService.srv"
    "srv/OntologeniusSparqlService.srv"
    "srv/OntologeniusSubscription.srv"
    "srv/OntologeniusUnsubscription.srv"
    DEPENDENCIES builtin_interfaces std_msgs)
endif()

# ##################################
# #   ROS specific configuration  ##
# ##################################
if($ENV{ROS_VERSION} STREQUAL "1")
  catkin_package(
    INCLUDE_DIRS include include/ontologenius/API
    LIBRARIES ontologenius_lib ontologenius_interface
    CATKIN_DEPENDS roscpp rospy std_msgs
    DEPENDS TINYXML2 OpenCV
    CFG_EXTRAS ontologenius_compatConfigExtras.cmake
  )
elseif($ENV{ROS_VERSION} STREQUAL "2")
  ament_export_dependencies(rosidl_default_runtime pluginlib)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
endif()

# ##########
# # Build ##
# ##########

# ##################################
# #   ROS independent libraries   ##
# ##################################

# # CORE
add_onto_library(ontologenius_ontoGraphs_lib
  src/core/ontoGraphs/Branchs/ClassBranch.cpp
  src/core/ontoGraphs/Branchs/ObjectPropertyBranch.cpp
  src/core/ontoGraphs/Branchs/DataPropertyBranch.cpp
  src/core/ontoGraphs/Branchs/IndividualBranch.cpp
  src/core/ontoGraphs/Branchs/ValuedNode.cpp
  src/core/ontoGraphs/Branchs/LiteralNode.cpp
  src/core/ontoGraphs/Graphs/ClassGraph.cpp
  src/core/ontoGraphs/Graphs/ObjectPropertyGraph.cpp
  src/core/ontoGraphs/Graphs/DataPropertyGraph.cpp
  src/core/ontoGraphs/Graphs/IndividualGraph.cpp
  src/core/ontoGraphs/Graphs/AnonymousClassGraph.cpp
  src/core/ontoGraphs/Graphs/RuleGraph.cpp
  src/core/ontoGraphs/Checkers/ClassChecker.cpp
  src/core/ontoGraphs/Checkers/RuleChecker.cpp
  src/core/ontoGraphs/Checkers/ObjectPropertyChecker.cpp
  src/core/ontoGraphs/Checkers/DataPropertyChecker.cpp
  src/core/ontoGraphs/Checkers/IndividualChecker.cpp
  src/core/ontoGraphs/Checkers/AnonymousClassChecker.cpp
  src/core/ontoGraphs/Ontology.cpp
  src/core/ontologyIO/OntologyLoader.cpp
  src/core/ontologyIO/OntologyReader.cpp
  src/core/ontologyIO/Turtle/OntologyTtlReader.cpp
  src/core/ontologyIO/Owl/OntologyOwlReader.cpp
  src/core/ontologyIO/Owl/OntologyOwlAnonymousReader.cpp
  src/core/ontologyIO/Owl/OntologyOwlRuleReader.cpp
  src/core/ontologyIO/Owl/OntologyOwlWriter.cpp
  src/core/ontologyIO/Owl/writers/NodeOwlWriter.cpp
  src/core/ontologyIO/Owl/writers/ClassOwlWriter.cpp
  src/core/ontologyIO/Owl/writers/ObjectPropertiesOwlWriter.cpp
  src/core/ontologyIO/Owl/writers/DataPropertiesOwlWriter.cpp
  src/core/ontologyIO/Owl/writers/AnnotationOwlWriter.cpp
  src/core/ontologyIO/Owl/writers/IndividualOwlWriter.cpp
  src/core/ontologyIO/Owl/writers/RuleOwlWriter.cpp
)
target_include_directories(ontologenius_ontoGraphs_lib
  PUBLIC
  ${TINYXML2_INCLUDE_DIRS}
)
target_link_libraries(ontologenius_ontoGraphs_lib
  PUBLIC
  ${TINYXML2_LIBRARIES}
  PRIVATE
  PkgConfig::CURL
)

add_onto_library(ontologenius_feeder_lib
  src/core/feeder/Feeder.cpp
  src/core/feeder/FeedStorage.cpp
  src/core/feeder/Versionor.cpp
  src/core/feeder/VersionNode.cpp
)
target_link_libraries(ontologenius_feeder_lib
  PUBLIC
  ontologenius_ontoGraphs_lib
)

# DRAWER
add_onto_library(ontologenius_drawer_lib
  src/graphical/versioning/TreeReader.cpp
  src/graphical/versioning/TreeDrawer.cpp
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

# # OPERATORS
add_onto_library(ontologenius_operators
  src/core/ontologyOperators/DifferenceFinder.cpp
  src/core/ontologyOperators/Sparql.cpp
  src/core/ontologyOperators/SparqlSolver.cpp
  src/core/ontologyOperators/SparqlUtils.cpp
)
target_link_libraries(ontologenius_operators
  PUBLIC
  ontologenius_feeder_lib
)

# ##################################
# #    ROS dependent libraries    ##
# ##################################
function(add_ros_library TARGET)
  if(NOT TARGET)
    message(FATAL_ERROR "Expected the target name as first argument")
  endif()

  if(NOT ARGN)
    message(FATAL_ERROR "Expected source file list after target name")
  endif()

  add_library(${TARGET} ${ARGN})

  if($ENV{ROS_VERSION} STREQUAL "1")
    add_dependencies(${TARGET} ontologenius_gencpp ${catkin_EXPORTED_TARGETS})
    target_include_directories(${TARGET} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PUBLIC ${catkin_LIBRARIES}) # PRIVATE has been removed fot ontoloGUI
  elseif($ENV{ROS_VERSION} STREQUAL "2")
    ament_target_dependencies(${TARGET} PUBLIC rclcpp pluginlib builtin_interfaces std_msgs)
    target_include_directories(${TARGET} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_link_libraries(${TARGET} PUBLIC ${cpp_typesupport_target})
    ament_export_libraries(${TARGET})
  endif()

  set_target_properties(${TARGET} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
  target_compile_options(${TARGET} PRIVATE $<$<CXX_COMPILER_ID:Clang>:-frelaxed-template-template-args>)
  target_compile_options(${TARGET} PUBLIC -DONTO_ROS_VERSION=$ENV{ROS_VERSION})
  target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
  target_enable_sanitizers(${TARGET})
endfunction(add_ros_library)

# # Compatibility
add_ros_library(ontologenius_compat
  src/compat/ros.cpp
)

# # PLUGINS
add_ros_library(ontologenius_plugin_lib
  src/core/reasoner/plugins/ReasonerInterface.cpp
  src/core/reasoner/Reasoners.cpp
  src/core/reasoner/ConfigReader.cpp
)
target_include_directories(ontologenius_plugin_lib
  PUBLIC ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(ontologenius_plugin_lib
  PUBLIC
  ontologenius_ontoGraphs_lib ${pluginlib_LIBRARIES}
)

add_ros_library(ontologenius_reasoner_plugin MODULE
  src/core/reasoner/plugins/ReasonerNone.cpp
  src/core/reasoner/plugins/ReasonerInverseOf.cpp
  src/core/reasoner/plugins/ReasonerSymmetric.cpp
  src/core/reasoner/plugins/ReasonerChain.cpp
  src/core/reasoner/plugins/ReasonerDictionary.cpp
  src/core/reasoner/plugins/ReasonerGeneralize.cpp
  src/core/reasoner/plugins/ReasonerRangeDomain.cpp
  src/core/reasoner/plugins/ReasonerAnonymous.cpp
  src/core/reasoner/plugins/ReasonerTransitivity.cpp
  src/core/reasoner/plugins/ReasonerRule.cpp
)
target_include_directories(ontologenius_reasoner_plugin
  PUBLIC ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(ontologenius_reasoner_plugin
  PUBLIC
  ontologenius_plugin_lib ontologenius_ontoGraphs_lib ${pluginlib_LIBRARIES}
)

# # API
add_ros_library(ontologenius_lib

  # Regular clients
  src/API/ontologenius/clients/ontologyClients/IndividualClient.cpp
  src/API/ontologenius/clients/ontologyClients/ObjectPropertyClient.cpp
  src/API/ontologenius/clients/ontologyClients/DataPropertyClient.cpp
  src/API/ontologenius/clients/ontologyClients/ClassClient.cpp
  src/API/ontologenius/clients/ontologyClients/OntologyClient.cpp
  src/API/ontologenius/clients/ReasonerClient.cpp
  src/API/ontologenius/clients/ActionClient.cpp
  src/API/ontologenius/clients/ManagerClient.cpp
  src/API/ontologenius/clients/ClientBase.cpp
  src/API/ontologenius/clients/SparqlClient.cpp

  # Indexed clients
  src/API/ontologenius/clientsIndex/ontologyClients/IndividualIndexClient.cpp
  src/API/ontologenius/clientsIndex/ontologyClients/ObjectPropertyIndexClient.cpp
  src/API/ontologenius/clientsIndex/ontologyClients/DataPropertyIndexClient.cpp
  src/API/ontologenius/clientsIndex/ontologyClients/ClassIndexClient.cpp
  src/API/ontologenius/clientsIndex/ontologyClients/OntologyIndexClient.cpp
  src/API/ontologenius/clientsIndex/ClientBaseIndex.cpp
  src/API/ontologenius/clientsIndex/SparqlIndexClient.cpp

  # Other
  src/API/ontologenius/FeederPublisher.cpp
  src/API/ontologenius/ConversionClient.cpp
  src/API/ontologenius/OntologyManipulator.cpp
  src/API/ontologenius/OntologyManipulatorIndex.cpp
  src/API/ontologenius/OntologiesManipulator.cpp
  src/API/ontologenius/PatternsSubscriber.cpp
)
target_include_directories(ontologenius_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ontologenius/API>
  $<INSTALL_INTERFACE:include/ontologenius/API>)
target_link_libraries(ontologenius_lib
  PUBLIC
  ontologenius_compat
  ontologenius_plugin_lib
) # ontologenius_interface

# # SUBSCRIPTION
add_ros_library(ontologenius_subscription_lib
  src/core/subscription/Subscription.cpp
  src/core/subscription/SubscriptionManager.cpp
  src/core/subscription/SubscriptionPattern.cpp
)
target_link_libraries(ontologenius_subscription_lib
  PUBLIC
  ontologenius_compat
  ontologenius_ontoGraphs_lib
)

# # INTERFACE
add_ros_library(ontologenius_interface
  src/interface/RosInterface.cpp
  src/interface/RosInterfaceStringHandlers.cpp
  src/interface/RosInterfaceIndexHandlers.cpp
)
target_link_libraries(ontologenius_interface
  PUBLIC
  ontologenius_compat
  ontologenius_feeder_lib
  ontologenius_plugin_lib
  ontologenius_operators
  ontologenius_subscription_lib
)

# ##################################
# #        C++ executables        ##
# ##################################
function(add_ros_executable TARGET)
  if(NOT TARGET)
    message(FATAL_ERROR "Expected the target name as first argument")
  endif()

  if(NOT ARGN)
    message(FATAL_ERROR "Expected source file list after target name")
  endif()

  add_executable(${TARGET} ${ARGN})

  if($ENV{ROS_VERSION} STREQUAL "1")
    target_include_directories(${TARGET} PUBLIC include PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PUBLIC ${catkin_LIBRARIES}) # PRIVATE has been removed fot ontoloGUI
  elseif($ENV{ROS_VERSION} STREQUAL "2")
    ament_target_dependencies(${TARGET} PUBLIC rclcpp pluginlib builtin_interfaces std_msgs)
    target_include_directories(${TARGET} PUBLIC include)
    target_link_libraries(${TARGET} PUBLIC ${cpp_typesupport_target})
  endif()

  set_target_properties(${TARGET} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
  target_compile_options(${TARGET} PUBLIC -DONTO_ROS_VERSION=$ENV{ROS_VERSION})
  target_compile_options(${TARGET} PRIVATE -Wall -Wextra)
  target_enable_sanitizers(${TARGET})
endfunction(add_ros_executable)

# # NODES
add_ros_executable(ontologenius_single src/nodes/ontologenius_single.cpp)
target_link_libraries(ontologenius_single PRIVATE ontologenius_interface)

add_ros_executable(ontologenius_multi src/nodes/ontologenius_multi.cpp)
target_link_libraries(ontologenius_multi PRIVATE ontologenius_interface)

add_ros_executable(ontologenius_draw src/graphical/versioning/main.cpp)
target_link_libraries(ontologenius_draw PRIVATE ontologenius_drawer_lib ontologenius_plugin_lib)

if($ENV{ROS_VERSION} STREQUAL "1")
  # # COMPARISONS

  # add_ros_executable(ontologenius_knowrob src/tests/Compare/knowrob_test.cpp)
  # target_link_libraries(ontologenius_knowrob PRIVATE ontologenius_interface)

  # add_ros_executable(ontologenius_oro src/tests/Compare/ORO_test.cpp)
  # target_link_libraries(ontologenius_oro PRIVATE ontologenius_interface ontologenius_lib)

  # add_ros_executable(ontologenius_ORO src/tests/Compare/ORO.cpp)
  # target_link_libraries(ontologenius_ORO PRIVATE ontologenius_lib ontologenius_plugin_lib)

  # # PERFORMANCES

  # add_ros_executable(ontologenius_deepcopy src/tests/Performances/deepcopy.cpp)
  # target_link_libraries(ontologenius_deepcopy PRIVATE ontologenius_lib)

  # add_ros_executable(ontologenius_find_tester src/tests/Performances/getNameVsFind.cpp)
  # target_link_libraries(ontologenius_find_tester PRIVATE ontologenius_lib ontologenius_plugin_lib ontologenius_interface)
  add_ros_executable(ontologenius_tester src/tests/Performances/tester.cpp)
  target_link_libraries(ontologenius_tester PRIVATE ontologenius_interface)

  add_ros_executable(ontologenius_container_tester src/tests/Performances/container_test.cpp)
  target_link_libraries(ontologenius_container_tester PRIVATE ontologenius_interface)

  # # FEATURES TESTS
  add_ros_executable(ontologenius_versioning src/tests/versioning.cpp)
  target_link_libraries(ontologenius_versioning PRIVATE ontologenius_lib)

  add_ros_executable(ontologenius_sparql src/tests/sparql.cpp)
  target_link_libraries(ontologenius_sparql PRIVATE ontologenius_interface)
endif()

# #############################################################################
# Qt Environment
# #############################################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# #############################################################################
# Sections
# #############################################################################
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

# #############################################################################
# Sources
# #############################################################################
set(QT_SOURCES
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

# #############################################################################
# Binaries
# #############################################################################
add_executable(ontoloGUI ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
set_target_properties(ontoloGUI PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(ontoloGUI PUBLIC -DONTO_ROS_VERSION=$ENV{ROS_VERSION})
target_link_libraries(ontoloGUI
  Qt5::Core
  Qt5::Widgets
  Qt5::PrintSupport
  ontologenius_lib)
target_include_directories(ontoloGUI PUBLIC include)

# ##################################
# #            Install            ##
# ##################################
if($ENV{ROS_VERSION} STREQUAL "1")
  # Generate CMake package config file
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    cmake/ontologenius_compatConfigIntermediate.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ontologenius_compatConfigIntermediate.cmake
    INSTALL_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
  )
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/ontologenius_compatConfigIntermediate.cmake
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
  )

  install(
    TARGETS ontologenius_compat
    EXPORT ontologenius_compatTargets
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  # Install the target file
  install(
    EXPORT ontologenius_compatTargets
    FILE ontologenius_compatTargets.cmake
    NAMESPACE ontologenius::
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
  )

  install(TARGETS
    ontoloGUI
    ontologenius_single
    ontologenius_multi
    ontologenius_draw
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(TARGETS
    ontologenius_lib
    ontologenius_interface
    ontologenius_compat
    ontologenius_operators
    ontologenius_feeder_lib
    ontologenius_plugin_lib
    ontologenius_ontoGraphs_lib
    ontologenius_reasoner_plugin
    ontologenius_subscription_lib
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

  install(DIRECTORY include/${PROJECT_NAME}/API/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

  install(DIRECTORY files/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/files
  )

  install(DIRECTORY configuration/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configuration
  )

  install(FILES reasoners_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} STREQUAL "2")
  install(TARGETS
    ontologenius_lib
    ontologenius_interface
    ontologenius_compat
    ontologenius_operators
    ontologenius_feeder_lib
    ontologenius_plugin_lib
    ontologenius_ontoGraphs_lib
    ontologenius_subscription_lib
    ontologenius_reasoner_plugin
    EXPORT ontologenius
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
    ontologenius_single
    ontologenius_multi
    ontologenius_draw
    ontoloGUI
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch/ros2/launch DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY include/ DESTINATION include)

  install(DIRECTORY files DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY docs DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY configuration DESTINATION share/${PROJECT_NAME}/)

  ament_export_targets(ontologenius)
endif()

# ##################################
# #            Tests              ##
# ##################################
if($ENV{ROS_VERSION} STREQUAL "1")
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(onto_api_connection_test test/api_connection.test src/tests/CI/api_connection_test.cpp)
    target_include_directories(onto_api_connection_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_api_connection_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_api_inheritage_test test/api_inheritage.test src/tests/CI/api_inheritage_test.cpp)
    target_include_directories(onto_api_inheritage_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_api_inheritage_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_api_relations_index_test test/api_relations_index.test src/tests/CI/api_relations_index_test.cpp)
    target_include_directories(onto_api_relations_index_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_api_relations_index_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_api_relations_test test/api_relations.test src/tests/CI/api_relations_test.cpp)
    target_include_directories(onto_api_relations_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_api_relations_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_feature_deep_copy_test test/feature_deep_copy.test src/tests/CI/feature_deep_copy_test.cpp)
    set_target_properties(onto_feature_deep_copy_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_include_directories(onto_feature_deep_copy_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_feature_deep_copy_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_feature_loading_test test/feature_loading.test src/tests/CI/feature_loading_test.cpp)
    target_include_directories(onto_feature_loading_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_feature_loading_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_feature_multi_test test/feature_multi.test src/tests/CI/feature_multi_test.cpp)
    target_include_directories(onto_feature_multi_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_feature_multi_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_feature_subscription_test test/feature_subscription.test src/tests/CI/feature_subscription_test.cpp)
    target_include_directories(onto_feature_subscription_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_feature_subscription_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_reasoning_anonymous_class_test test/reasoning_anonymous_class.test src/tests/CI/reasoning_anonymous_class_test.cpp)
    set_target_properties(onto_reasoning_anonymous_class_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_include_directories(onto_reasoning_anonymous_class_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_reasoning_anonymous_class_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_reasoning_chain_test test/reasoning_chain.test src/tests/CI/reasoning_chain_test.cpp)
    set_target_properties(onto_reasoning_chain_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_include_directories(onto_reasoning_chain_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_reasoning_chain_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_reasoning_mechanism_test test/reasoning_mechanism.test src/tests/CI/reasoning_mechanism_test.cpp)
    set_target_properties(onto_reasoning_mechanism_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_include_directories(onto_reasoning_mechanism_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_reasoning_mechanism_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_reasoning_transitivity_test test/reasoning_transitivity.test src/tests/CI/reasoning_transitivity_test.cpp)
    set_target_properties(onto_reasoning_transitivity_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_include_directories(onto_reasoning_transitivity_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_reasoning_transitivity_test ontologenius_lib ${catkin_LIBRARIES})

    add_rostest_gtest(onto_reasoning_rule_test test/reasoning_rule.test src/tests/CI/reasoning_rule_test.cpp)
    set_target_properties(onto_reasoning_rule_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_include_directories(onto_reasoning_rule_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(onto_reasoning_rule_test ontologenius_lib ${catkin_LIBRARIES})
  endif()

elseif($ENV{ROS_VERSION} STREQUAL "2")
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(feature_deep_copy_test src/tests/CI/feature_deep_copy_test.cpp TIMEOUT 10)
    set_target_properties(feature_deep_copy_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    target_include_directories(feature_deep_copy_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(feature_deep_copy_test ontologenius_lib ${catkin_LIBRARIES})
  endif()
endif()

# ##################################
# #            ROS2               ##
# ##################################
if($ENV{ROS_VERSION} STREQUAL "2")
  pluginlib_export_plugin_description_file(ontologenius reasoners_plugins_ros2.xml)

  ament_export_include_directories(include include/ontologenius/API)
  ament_package()
endif()
