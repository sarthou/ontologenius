#include "ontologenius/compat/ros.h"

namespace ontologenius::compat::ros
{
    Node& Node::get()
    {
        static Node compat_node("ontologenius");
        return compat_node;
    }

    bool Node::ok()
    {
#if ROS_VERSION == 1
        return ros::ok();
#elif ROS_VERSION == 2
        return get().running_;
#endif
    }

    void Node::init(int argc, char **argv)
    {
#if ROS_VERSION == 1
        ros::init(argc, argv, name_);
#elif ROS_VERSION == 2
        rclcpp::init(argc, argv);
#endif
    }

    void Node::shutdown()
    {
#if ROS_VERSION == 1
        // todo: handle ROS1 shutdown
#elif ROS_VERSION == 2
        rclcpp::shutdown();
#endif
    }

    Time Node::current_time()
    {
#if ROS_VERSION == 1
        return ros::Time::now();
#elif ROS_VERSION == 2
        return handle_->now();
#endif
    }

    Node::Node(const std::string& node_name)
        : name_(node_name),
#if ROS_VERSION == 2
          handle_(std::make_shared<rclcpp::Node>(node_name)),
#endif
          running_(true)
    {
        // todo: should we put something here?
    }

    Node::~Node() {

    }
}