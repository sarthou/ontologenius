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

add_library(ontologenius_compat "src/compat/ros.cpp")
add_ros_deps(ontologenius_compat)

target_include_directories(ontologenius_compat PUBLIC include/)

install(TARGETS ontologenius_compat DESTINATION lib/${PROJECT_NAME})

add_library(ontologenius_interface
        "src/interface/RosInterface.cpp"
        "src/interface/RosInterfaceIndexHandlers.cpp"
        "src/interface/RosInterfaceStringHandlers.cpp")
add_ros_deps(ontologenius_interface)

target_include_directories(ontologenius_interface PUBLIC include/)

add_executable(n1pe
        "src/n1pe/main.cpp")

add_ros_deps(n1pe)
target_include_directories(n1pe PUBLIC include/)
target_link_libraries(n1pe "ontologenius_compat")

install(TARGETS n1pe DESTINATION lib/${PROJECT_NAME})

ament_package()
