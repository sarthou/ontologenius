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

namespace std_msgs_compat = std_msgs::msg;

namespace ontologenius::compat
{
#if ROS_VERSION == 1
    using namespace ::ontologenius;

    // todo: RequestType, ResponseType, make_request, make_response
    // Probably should wrap these in a class with overloaded -> operator & overloaded T& operator
#elif ROS_VERSION == 2
    using namespace ::ontologenius::msg;
    using namespace ::ontologenius::srv;

    template <typename T>
    using RawRequestType = typename T::Request;

    template <typename T>
    using RawResponseType = typename T::Response;

    template <typename T>
    using RequestType = std::shared_ptr<typename T::Request>;

    template <typename T>
    using ResponseType = std::shared_ptr<typename T::Response>;

    template <typename T, typename Result_ = typename T::Request>
    inline auto make_request() { return std::make_shared<Result_>(); }

    template <typename T, typename Result_ = typename T::Response>
    inline auto make_response() { return std::make_shared<Result_>(); }

    // template <typename T, typename Result_ = typename T::>
#endif

    namespace ros
    {
#if ROS_VERSION == 1
        inline void spin_once() {
            ros::spinOnce();
        }

        inline void wait_for_service(const std::string& service_name) {
            ros::service::waitForService(service_name);
        }

        // todo: create ServiceWrapper class with implicit constructor taking T& & with an overloaded -> operator

        using Rate = ros::Rate;
        using Time = ros::Time;
#elif ROS_VERSION == 2
        inline void spin_once() {
            // do nothing
        }

        inline bool wait_for_service(const std::string& service_name, int32_t timeout = -1) {
            return true; // todo
        }

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

            static void init(int argc, char **argv, const std::string& node_name);
            static void shutdown();

            void spin();

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
                // todo
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
            enum class Result { SUCCESSFUL, SUCCESSFUL_WITH_RETRIES, FAILURE };

            Client(const std::string& service_name) {
                auto& node = Node::get();
                auto& node_handle = node.handle_;

#if ROS_VERSION == 1
                handle_ = node_handle.serviceClient<ontologenius::compat::OntologeniusService>(service_name, true);
#elif ROS_VERSION == 2
                handle_ = node_handle->create_client<T>(service_name);
#endif
            }

            Result call(const ontologenius::compat::RequestType<T>& req, ontologenius::compat::ResponseType<T>& res) {
                auto& node = Node::get();
                auto& node_handle = node.handle_;

                auto attempts = 0;
                auto result = Result::FAILURE;

#if ROS_VERSION == 1
                T msg;
                msg.request = req;

                // todo: finish this
#elif ROS_VERSION == 2
                while ((result == Result::FAILURE) && attempts < 3)
                {
                    auto promise = handle_->async_send_request(req);

                    switch (rclcpp::spin_until_future_complete(node_handle, promise)) {
                        case rclcpp::FutureReturnCode::SUCCESS: {
                            result = (attempts > 0) ? Result::SUCCESSFUL_WITH_RETRIES : Result::SUCCESSFUL;
                            res = promise.get();
                            break;
                        }
                        // todo: maybe treat these differently?
                        case rclcpp::FutureReturnCode::INTERRUPTED:
                        case rclcpp::FutureReturnCode::TIMEOUT: {
                            attempts++;
                            break;
                        }
                    }
                }
#endif
                return result;
            }
        private:
#if ROS_VERSION == 1
            ros::ServiceClient handle_;
#elif ROS_VERSION == 2
            typename rclcpp::Client<T>::SharedPtr handle_;
#endif
        };
    }
}

#endif // COMPAT_ROS_H