#include "ontologenius/compat/ros.h"

#include <string>

namespace ontologenius::compat::onto_ros {

  std::string ros_node_name = "OntoRos";

  Node& Node::get()
  {
    static Node node_(ros_node_name);
    return node_;
  }

  bool Node::ok()
  {
#if ONTO_ROS_VERSION == 1
    return ros::ok();
#elif ONTO_ROS_VERSION == 2
    return rclcpp::ok();
#endif
  }

  void Node::init(int argc, char** argv, const std::string& node_name)
  {
    ros_node_name = node_name;

#if ONTO_ROS_VERSION == 1
    ros::init(argc, argv, ros_node_name);
#elif ONTO_ROS_VERSION == 2
    rclcpp::init(argc, argv);
#endif
  }

  void Node::shutdown()
  {
#if ONTO_ROS_VERSION == 1
    ros::shutdown();
#elif ONTO_ROS_VERSION == 2
    rclcpp::shutdown();
#endif
  }

  void Node::spin()
  {
#if ONTO_ROS_VERSION == 1
    ros::spin();
#elif ONTO_ROS_VERSION == 2
    // rclcpp::spin(handle_);
#endif
  }

  Time Node::current_time()
  {
#if ONTO_ROS_VERSION == 1
    return Time(ros::Time::now());
#elif ONTO_ROS_VERSION == 2
    return Time(handle_->now());
#endif
  }

  Node::Node(const std::string& node_name) : name_(node_name),
#if ONTO_ROS_VERSION == 2
                                             handle_(std::make_shared<rclcpp::Node>(node_name)),
#endif
                                             running_(true)
  {
    // todo: should we put something here?
#if ONTO_ROS_VERSION == 2
    ros_thread_ = std::thread([this]() { rclcpp::spin(handle_); });
#endif
  }

} // namespace ontologenius::compat::onto_ros