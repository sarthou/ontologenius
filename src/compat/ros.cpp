#include "ontologenius/compat/ros.h"

namespace ontologenius::compat::ros
{
    namespace implementation {
        std::mutex spin_mutex;
    }

    std::string node_name__;

    Node& Node::get()
    {
        static Node compat_node(node_name__);
        return compat_node;
    }

    bool Node::ok()
    {
#if ROS_VERSION == 1
        return ros::ok();
#elif ROS_VERSION == 2
        return rclcpp::ok();
#endif
    }

    void Node::init(int argc, char **argv, const std::string& node_name)
    {
        fmt::print("[debug] Node::init\n");

        node_name__ = node_name;

#if ROS_VERSION == 1
        ros::init(argc, argv, name_);
#elif ROS_VERSION == 2
        rclcpp::init(argc, argv);
#endif
    }

    void Node::shutdown()
    {
        fmt::print("[debug] Node::shutdown\n");

#if ROS_VERSION == 1
        // todo: handle ROS1 shutdown
#elif ROS_VERSION == 2
        rclcpp::shutdown();
#endif
    }


    void Node::spin()
    {
        fmt::print("[debug] Node::spin\n");

#if ROS_VERSION == 1
        // todo: handle ROS1 spin
#elif ROS_VERSION == 2
        rclcpp::spin(handle_);
#endif
    }

    Time Node::current_time()
    {
        fmt::print("[debug] Node::current_time\n");

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
        fmt::print("[debug] Node::Node\n");

        // todo: should we put something here?
    }

    Node::~Node() {

    }
}