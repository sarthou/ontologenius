cmake_minimum_required(VERSION 3.8)
project(ontologenius)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_search_module(CURL REQUIRED libcurl IMPORTED_TARGET GLOBAL)

rosidl_generate_interfaces(ontologenius
        "msg/OntologeniusTimestamp.msg"
        "msg/OntologeniusExplanation.msg"
        "msg/OntologeniusSparqlIndexResponse.msg"
        "msg/OntologeniusSparqlResponse.msg"
        "msg/OntologeniusStampedString.msg"
        "srv/OntologeniusConversion.srv"
        "srv/OntologeniusIndexService.srv"
        "srv/OntologeniusService.srv"
        "srv/OntologeniusSparqlIndexService.srv"
        "srv/OntologeniusSparqlService.srv"
        DEPENDENCIES builtin_interfaces std_msgs)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

function(add_ros_deps TARGET)
    target_link_libraries(${TARGET} ${cpp_typesupport_target})
    ament_target_dependencies(${TARGET} rclcpp pluginlib builtin_interfaces std_msgs)

    target_include_directories(${TARGET} PUBLIC
            ${pluginlib_INCLUDE_DIRS}
            /opt/ros/humble/include/pluginlib/) # todo figure out why pluginlib is not inside pluginlib_INCLUDE_DIRS
endfunction()

# --------------------------------------------------------------

add_library(ontologenius_compat "src/compat/ros.cpp")
add_ros_deps(ontologenius_compat)

target_include_directories(ontologenius_compat PUBLIC include/)

# --------------------------------------------------------------

add_library(ontologenius_operators
        "src/core/ontologyOperators/differenceFinder.cpp"
        "src/core/ontologyOperators/Sparql.cpp"
        "src/core/ontologyOperators/SparqlSolver.cpp"
        "src/core/ontologyOperators/SparqlUtils.cpp")

target_link_libraries(ontologenius_operators
        PUBLIC
        ontologenius_core_lib)

# --------------------------------------------------------------

add_library(ontologenius_interface
        "src/interface/RosInterface.cpp"
        "src/interface/RosInterfaceIndexHandlers.cpp"
        "src/interface/RosInterfaceStringHandlers.cpp")
add_ros_deps(ontologenius_interface)

target_link_libraries(ontologenius_interface
        ontologenius_operators)

target_include_directories(ontologenius_interface PUBLIC include/)

# --------------------------------------------------------------

add_library(ontologenius_lib
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
        "src/API/ontologenius/OntologyManipulatorIndex.cpp")
add_ros_deps(ontologenius_lib)
target_link_libraries(ontologenius_lib
        ontologenius_compat
        ontologenius_interface)

target_include_directories(ontologenius_lib PUBLIC include/)

# --------------------------------------------------------------

add_library(ontologenius_ontoGraphs_lib
        "src/core/ontoGraphs/Branchs/ClassBranch.cpp"
        "src/core/ontoGraphs/Branchs/ObjectPropertyBranch.cpp"
        "src/core/ontoGraphs/Branchs/DataPropertyBranch.cpp"
        "src/core/ontoGraphs/Branchs/IndividualBranch.cpp"
        "src/core/ontoGraphs/Branchs/ValuedNode.cpp"
        "src/core/ontoGraphs/Branchs/LiteralNode.cpp"
        "src/core/ontoGraphs/Graphs/ClassGraph.cpp"
        "src/core/ontoGraphs/Graphs/ObjectPropertyGraph.cpp"
        "src/core/ontoGraphs/Graphs/DataPropertyGraph.cpp"
        "src/core/ontoGraphs/Graphs/IndividualGraph.cpp"
        "src/core/ontoGraphs/Checkers/ClassChecker.cpp"
        "src/core/ontoGraphs/Checkers/ObjectPropertyChecker.cpp"
        "src/core/ontoGraphs/Checkers/DataPropertyChecker.cpp"
        "src/core/ontoGraphs/Checkers/IndividualChecker.cpp"
        "src/core/ontoGraphs/Ontology.cpp"
        "src/core/ontologyIO/OntologyLoader.cpp"
        "src/core/ontologyIO/OntologyReader.cpp"
        "src/core/ontologyIO/Turtle/OntologyTtlReader.cpp"
        "src/core/ontologyIO/Owl/OntologyOwlReader.cpp"
        "src/core/ontologyIO/Owl/OntologyOwlWriter.cpp"
        "src/core/ontologyIO/Owl/writers/NodeOwlWriter.cpp"
        "src/core/ontologyIO/Owl/writers/ClassOwlWriter.cpp"
        "src/core/ontologyIO/Owl/writers/ObjectPropertiesOwlWriter.cpp"
        "src/core/ontologyIO/Owl/writers/DataPropertiesOwlWriter.cpp"
        "src/core/ontologyIO/Owl/writers/AnnotationOwlWriter.cpp"
        "src/core/ontologyIO/Owl/writers/IndividualOwlWriter.cpp")

target_include_directories(ontologenius_ontoGraphs_lib PUBLIC
        ${TinyXML_INCLUDE_DIRS}
        include/)

target_link_libraries(ontologenius_ontoGraphs_lib PUBLIC
        ${TinyXML_LIBRARIES} PkgConfig::CURL)

# --------------------------------------------------------------

add_library(ontologenius_core_lib
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
        include/
        /opt/ros/humble/include/pluginlib/) # todo figure out why pluginlib is not inside pluginlib_INCLUDE_DIRS

target_link_libraries(ontologenius_core_lib PUBLIC
        ontologenius_ontoGraphs_lib
        ${TinyXML2_LIBRARIES}
        ${pluginlib_LIBRARIES})

# --------------------------------------------------------------




add_executable(n1pe
        "src/nodes/ontologenius_single.cpp")

add_ros_deps(n1pe)
target_link_libraries(n1pe
        ontologenius_compat
        ontologenius_interface
        ontologenius_lib
        ontologenius_ontoGraphs_lib
        ontologenius_operators
        ontologenius_core_lib
        fmt)
target_include_directories(n1pe PUBLIC include/)

install(TARGETS n1pe DESTINATION lib/${PROJECT_NAME})

ament_package()
