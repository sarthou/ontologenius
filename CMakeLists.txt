cmake_minimum_required(VERSION 3.8)

project(ontologenius)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra) # -Wpedantic
endif()

find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(ontologenius
        "msg/Timestamp.msg"
        "msg/OntologeniusExplanation.msg"
        "msg/OntologeniusSparqlIndexResponse.msg"
        "msg/OntologeniusSparqlResponse.msg"
        "msg/StampedString.msg"
        "srv/OntologeniusConversion.srv"
        "srv/OntologeniusIndexService.srv"
        "srv/OntologeniusService.srv"
        "srv/OntologeniusSparqlIndexService.srv"
        "srv/OntologeniusSparqlService.srv"
        DEPENDENCIES builtin_interfaces std_msgs)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
find_package(PkgConfig REQUIRED)

pkg_search_module(CURL REQUIRED libcurl IMPORTED_TARGET GLOBAL)

function(add_onto_base_lib TARGET)
    if (NOT TARGET)
        message(FATAL_ERROR "Expected the target name as the first argument")
    endif()

    if (NOT ARGN)
        message(FATAL_ERROR "Expected source files after the target name")
    endif()

    file(GLOB_RECURSE SOURCES ${ARGN})
    add_library(${TARGET} ${SOURCES})
    target_include_directories(${TARGET} PUBLIC include/)
    # target_enable_sanitizers(${TARGET})
endfunction(add_onto_base_lib)

function(add_onto_lib TARGET)
    add_onto_base_lib(${TARGET} ${ARGN})
    ament_target_dependencies(${TARGET} pluginlib rclcpp builtin_interfaces std_msgs)
endfunction(add_onto_lib)

function(add_onto_base_executable TARGET MAIN_SOURCE_FILE)
    if (NOT TARGET)
        message(FATAL_ERROR "Expected the target name as the first argument")
    endif()

    add_executable(${TARGET} ${MAIN_SOURCE_FILE})
    target_include_directories(${TARGET} PUBLIC include/)
    # target_enable_sanitizers(${TARGET})
endfunction(add_onto_base_executable)

function(add_onto_executable TARGET MAIN_SOURCE_FILE)
    add_onto_base_executable(${TARGET} ${MAIN_SOURCE_FILE})
    ament_target_dependencies(${TARGET} pluginlib rclcpp builtin_interfaces std_msgs)
    # target_link_libraries(${TARGET} PUBLIC ${TODO_FIGURE_THIS_OUT})
endfunction(add_onto_executable)

# ------------------------------------------
add_onto_base_lib(ontologenius_ontoGraphs_lib
        src/core/ontoGraphs/*.cpp
        src/core/ontologyIO/*.cpp)

target_include_directories(ontologenius_ontoGraphs_lib PUBLIC
        ${TinyXML_INCLUDE_DIRS})

target_link_libraries(ontologenius_ontoGraphs_lib PUBLIC
        ${TinyXML_LIBRARIES} PkgConfig::CURL)
# ------------------------------------------
add_onto_base_lib(ontologenius_core_lib
        "src/core/feeder/Feeder.cpp"
        "src/core/feeder/FeedStorage.cpp"
        "src/core/feeder/Versionor.cpp"
        "src/core/feeder/Version_node.cpp"
        "src/core/reasoner/plugins/ReasonerChain.cpp"
        "src/core/reasoner/plugins/ReasonerDictionary.cpp"
        "src/core/reasoner/plugins/ReasonerGeneralize.cpp"
        "src/core/reasoner/plugins/ReasonerInterface.cpp"
        "src/core/reasoner/plugins/ReasonerInverseOf.cpp"
        "src/core/reasoner/plugins/ReasonerNone.cpp"
        "src/core/reasoner/plugins/ReasonerRangeDomain.cpp"
        "src/core/reasoner/plugins/ReasonerSymmetric.cpp"
        "src/core/reasoner/ConfigReader.cpp"
        "src/core/reasoner/Reasoners.cpp")

target_include_directories(ontologenius_core_lib PUBLIC
        ${pluginlib_INCLUDE_DIRS}
        /opt/ros/humble/include/pluginlib/) # todo figure out why pluginlib is not inside pluginlib_INCLUDE_DIRS

target_link_libraries(ontologenius_core_lib PUBLIC
        ontologenius_ontoGraphs_lib
        ${pluginlib_LIBRARIES})
# ------------------------------------------
add_onto_base_lib(ontologenius_drawer_lib
        "src/graphical/versioning/TreeDrawer.cpp"
        "src/graphical/versioning/TreeReader.cpp")

target_include_directories(ontologenius_drawer_lib PUBLIC
        ${OpenCV_INCLUDE_DIRS})

target_link_libraries(ontologenius_drawer_lib PUBLIC
        ${OpenCV_LIBS}
        ontologenius_ontoGraphs_lib)
# ------------------------------------------
add_onto_base_lib(ontologenius_reasoner_plugin
        "src/core/reasoner/plugins/ReasonerNone.cpp"
        "src/core/reasoner/plugins/ReasonerInverseOf.cpp"
        "src/core/reasoner/plugins/ReasonerSymmetric.cpp"
        "src/core/reasoner/plugins/ReasonerChain.cpp"
        "src/core/reasoner/plugins/ReasonerDictionary.cpp"
        "src/core/reasoner/plugins/ReasonerGeneralize.cpp"
        "src/core/reasoner/plugins/ReasonerRangeDomain.cpp")

target_include_directories(ontologenius_reasoner_plugin PUBLIC
        ${pluginlib_INCLUDE_DIRS}
        "/opt/ros/humble/include/pluginlib/") # todo figure out why pluginlib is not inside pluginlib_INCLUDE_DIRS

target_link_libraries(ontologenius_reasoner_plugin PUBLIC
        ${pluginlib_LIBRARIES}
        ontologenius_core_lib)
# ------------------------------------------
add_onto_base_lib(ontologenius_operators
        "src/core/ontologyOperators/differenceFinder.cpp"
        "src/core/ontologyOperators/Sparql.cpp"
        "src/core/ontologyOperators/SparqlSolver.cpp"
        "src/core/ontologyOperators/SparqlUtils.cpp")

target_link_libraries(ontologenius_operators PUBLIC
        ontologenius_core_lib)
# ------------------------------------------
add_onto_lib(ontologenius_lib
        # Regular clients
        "src/API/ontologenius/clients/ontologyClients/ClassClient.cpp"
        "src/API/ontologenius/clients/ontologyClients/DataPropertyClient.cpp"
        "src/API/ontologenius/clients/ontologyClients/IndividualClient.cpp"
        "src/API/ontologenius/clients/ontologyClients/ObjectPropertyClient.cpp"
        "src/API/ontologenius/clients/ontologyClients/OntologyClient.cpp"
        "src/API/ontologenius/clients/ActionClient.cpp"
        "src/API/ontologenius/clients/ClientBase.cpp"
        "src/API/ontologenius/clients/ManagerClient.cpp"
        "src/API/ontologenius/clients/ReasonerClient.cpp"
        "src/API/ontologenius/clients/SparqlClient.cpp"
        # Indexed clients
        "src/API/ontologenius/clientsIndex/ontologyClients/ClassIndexClient.cpp"
        "src/API/ontologenius/clientsIndex/ontologyClients/DataPropertyIndexClient.cpp"
        "src/API/ontologenius/clientsIndex/ontologyClients/IndividualIndexClient.cpp"
        "src/API/ontologenius/clientsIndex/ontologyClients/ObjectPropertyIndexClient.cpp"
        "src/API/ontologenius/clientsIndex/ontologyClients/OntologyIndexClient.cpp"
        "src/API/ontologenius/clientsIndex/ClientBaseIndex.cpp"
        "src/API/ontologenius/clientsIndex/SparqlIndexClient.cpp"
        # Other
        "src/API/ontologenius/ConversionClient.cpp"
        "src/API/ontologenius/FeederPublisher.cpp"
        "src/API/ontologenius/OntologiesManipulator.cpp"
        "src/API/ontologenius/OntologyManipulator.cpp"
        "src/API/ontologenius/OntologyManipulatorIndex.cpp)")

rosidl_target_interfaces(ontologenius_lib
        ontologenius "rosidl_typesupport_cpp")
# ------------------------------------------
add_onto_lib(ontologenius_interface
        "src/interface/RosInterface.cpp"
        "src/interface/RosInterfaceIndexHandlers.cpp"
        "src/interface/RosInterfaceStringHandlers.cpp")

rosidl_target_interfaces(ontologenius_interface
        ontologenius "rosidl_typesupport_cpp")
# ------------------------------------------
add_onto_base_executable(ontologenius_draw
        "src/graphical/versioning/main.cpp")

target_link_libraries(ontologenius_draw PUBLIC
        ontologenius_core_lib
        ontologenius_drawer_lib)
# ------------------------------------------
add_onto_executable(ontologenius_single
        src/nodes/ontologenius_single.cpp)

target_link_libraries(ontologenius_single
        ontologenius_interface)
# ------------------------------------------
add_onto_executable(ontologenius_multi
        src/nodes/ontologenius_multi.cpp)

target_link_libraries(ontologenius_multi
        ontologenius_interface)
# ------------------------------------------
add_onto_executable(ontologenius_tester
        src/tests/Performances/tester.cpp)

target_link_libraries(ontologenius_tester
        ontologenius_core_lib)
# ------------------------------------------
add_onto_executable(ontologenius_container_tester
        src/tests/Performances/container_test.cpp)

target_link_libraries(ontologenius_container_tester
        ontologenius_core_lib)
# ------------------------------------------
add_onto_executable(ontologenius_config_tester
        src/tests/config_reader_test.cpp)

target_link_libraries(ontologenius_config_tester
        ontologenius_core_lib)
# ------------------------------------------
add_onto_executable(ontologenius_versioning
        src/tests/versioning.cpp)

target_link_libraries(ontologenius_versioning
        ontologenius_core_lib)
# ------------------------------------------
add_onto_executable(ontologenius_sparql
        src/tests/sparql.cpp)

target_link_libraries(ontologenius_sparql
        ontologenius_core_lib)
# ------------------------------------------

ament_package()
