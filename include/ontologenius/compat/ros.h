#ifndef COMPAT_ROS_H
#define COMPAT_ROS_H

#if ROS_VERSION == 1
#include <ros/ros.h>
#include <ros/callback_queue.h>

// Commonly used built-in interfaces
#include <std_msgs/String.h>

// User-defined message interfaces
#include <ontologenius/OntologeniusExplanation.h>
#include <ontologenius/OntologeniusSparqlIndexResponse.h>
#include <ontologenius/OntologeniusSparqlResponse.h>
#include <ontologenius/OntologeniusStampedString.h>
#include <ontologenius/OntologeniusTimestamp.h>

// User-defined service interfaces
#include <ontologenius/OntologeniusConversion.hpp>
#include <ontologenius/OntologeniusIndexService.hpp>
#include <ontologenius/OntologeniusService.hpp>
#include <ontologenius/OntologeniusSparqlIndexService.hpp>
#include <ontologenius/OntologeniusService.hpp>
#elif ROS_VERSION == 2
#include <rclcpp/rclcpp.hpp>

// Commonly used built-in interfaces
#include <std_msgs/msg/string.hpp>

// User-defined message interfaces
#include <ontologenius/msg/ontologenius_explanation.hpp>
#include <ontologenius/msg/ontologenius_sparql_index_response.hpp>
#include <ontologenius/msg/ontologenius_sparql_response.hpp>
#include <ontologenius/msg/ontologenius_stamped_string.hpp>
#include <ontologenius/msg/ontologenius_timestamp.hpp>

// User-defined service interfaces
#include <ontologenius/srv/ontologenius_conversion.hpp>
#include <ontologenius/srv/ontologenius_service.hpp>
#include <ontologenius/srv/ontologenius_index_service.hpp>
#include <ontologenius/srv/ontologenius_sparql_service.hpp>
#include <ontologenius/srv/ontologenius_sparql_index_service.hpp>
#endif

#include <string>
#include <memory>
#include <functional>
#include <map>
#include <functional>

namespace std_msgs_compat = std_msgs::msg;

namespace ontologenius::compat
{
#if ROS_VERSION == 1
    using namespace ontologenius;
#elif ROS_VERSION == 2
    using namespace ::ontologenius::msg;
    using namespace ::ontologenius::srv;
#endif

    namespace ros
    {
#if ROS_VERSION == 1
        // todo: create ServiceWrapper class with implicit constructor taking T& & with an overloaded -> operator

        template <typename T>
        using ServiceWrapper = T;

        using Rate = ros::Rate;
        using Time = ros::Time;
#elif ROS_VERSION == 2
        template <typename T>
        using ServiceWrapper = std::shared_ptr<T>;

        using Rate = rclcpp::Rate;
        using Time = rclcpp::Time;

        using namespace ::ontologenius::msg;
        using namespace ::ontologenius::srv;
#endif

        template <typename T>
        class Publisher;

        template <typename T>
        class Subscriber;

        template <typename T>
        class Service;

        template <typename T>
        class Client;

        class Node
        {
        public:
            static Node& get();
            static bool ok();

            void init(int argc, char **argv);
            void shutdown();

            Time current_time();
        private:
            explicit Node(const std::string& node_name);

            const std::string name_;

#if ROS_VERSION == 1
            ros::NodeHandle handle_;
            ros::CallbackQueue callback_queue_;
#elif ROS_VERSION == 2
            rclcpp::Node::SharedPtr handle_;
#endif

            bool running_;
        public:
            Node(Node& other) = delete;
            Node(Node&& other) = delete;
            ~Node();

            template <typename T>
            friend class Publisher;

            template <typename T>
            friend class Subscriber;

            template <typename T>
            friend class Service;

            template <typename T>
            friend class Client;
        };

        template <typename T>
        class Publisher {
        public:
            Publisher(const std::string& topic_name, std::size_t queue_size) {
                auto& node = Node::get();
                auto& node_handle = node.handle_;

#if ROS_VERSION == 1
                handle_ = node_handle.advertise<T>(topic_name, queue_size);
#elif ROS_VERSION == 2
                (void) queue_size;
                handle_ = node_handle->create_publisher<T>(topic_name, 10);
#endif
            }

            void publish(const T& message) {
                auto& node = Node::get();
                auto& node_handle = node.handle_;

#if ROS_VERSION == 1

#elif ROS_VERSION == 2
                handle_->publish(message);
#endif
            }
        private:
#if ROS_VERSION == 1
            ros::Publisher handle_;
#elif ROS_VERSION == 2
            typename rclcpp::Publisher<T>::SharedPtr handle_;
#endif
        };

        template <typename T>
        class Subscriber {
        public:
            template<typename Ta, typename Tb>
            Subscriber(const std::string& topic_name, std::size_t queue_size, Ta&& callback, Tb&& ptr) {
                auto& node = Node::get();
                auto& node_handle = node.handle_;

#if ROS_VERSION == 1
                handle_ = node_handle.subscribe(topic_name, queue_size, callback, ptr);
#elif ROS_VERSION == 2
                (void) queue_size;
                handle_ = node_handle->create_subscription<T>(topic_name, 10, std::bind(std::forward<Ta>(callback), ptr, std::placeholders::_1));
#endif
            }
        private:
#if ROS_VERSION == 1
            ros::Subscriber handle_;
#elif ROS_VERSION == 2
            typename rclcpp::Subscription<T>::SharedPtr handle_;
#endif
        };

        template <typename T>
        class Service
        {
        public:
            template<typename Ta, typename Tb>
            Service(const std::string& service_name, Ta&& callback, Tb&& ptr) {
                auto& node = Node::get();
                auto& node_handle = node.handle_;

#if ROS_VERSION == 1
                handle_ = node_handle.advertiseService(service_name, callback, ptr);
#elif ROS_VERSION == 2
                handle_ = node_handle->create_service<T>(service_name, std::bind(std::forward<Ta>(callback), ptr, std::placeholders::_1, std::placeholders::_2));
#endif
            }
        private:
#if ROS_VERSION == 1
            ros::ServiceServer handle_;
#elif ROS_VERSION == 2
            typename rclcpp::Service<T>::SharedPtr handle_;
#endif
        };

        template <typename T>
        class Client
        {
        public:
            Client(const std::string& service_name) {
                auto& node = Node::get();
                auto& node_handle = node.handle_;

#if ROS_VERSION == 1

#elif ROS_VERSION == 2

#endif
            }
        private:
#if ROS_VERSION == 1
            ros::ServiceClient service_;
#elif ROS_VERSION == 2
            typename rclcpp::Client<T>::SharedPtr p_service_;
#endif
        };
    }
}

#endif // COMPAT_ROS_H