#ifndef COMPAT_ROS_H
#define COMPAT_ROS_H

#if ONTO_ROS_VERSION == 1
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
#include <ontologenius/OntologeniusConversion.h>
#include <ontologenius/OntologeniusIndexService.h>
#include <ontologenius/OntologeniusService.h>
#include <ontologenius/OntologeniusSparqlIndexService.h>
#include <ontologenius/OntologeniusSparqlService.h>
#include <ontologenius/OntologeniusSparqlIndexService.h>

namespace std_msgs_compat = std_msgs;

#elif ONTO_ROS_VERSION == 2
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

namespace std_msgs_compat = std_msgs::msg;

#endif

#include <string>
#include <memory>
#include <functional>
#include <map>
#include <mutex>

namespace ontologenius::compat
{
#if ONTO_ROS_VERSION == 1
  using namespace ::ontologenius;

  template <typename T>
  using RawRequestType = typename T::Request;

  template <typename T>
  using RawResponseType = typename T::Response;

  template <typename T>
  using RequestType = typename T::Request;

  template <typename T>
  using ResponseType = typename T::Response;

  template <typename T, typename Request_ = typename T::Request>
  inline auto make_request() { return Request_(); }

  template <typename T, typename Response_ = typename T::Response>
  inline auto make_response() { return Response_(); }

  // todo: RequestType, ResponseType

#elif ONTO_ROS_VERSION == 2
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

  template <typename T, typename Request_ = typename T::Request>
  inline auto make_request() { return std::make_shared<Request_>(); }

  template <typename T, typename Response_ = typename T::Response>
  inline auto make_response() { return std::make_shared<Response_>(); }

  // template <typename T, typename Result_ = typename T::>
#endif

namespace onto_ros
{

#if ONTO_ROS_VERSION == 1
  template <typename T>
  using ServiceWrapper = T;

  template <typename T>
  using MessageWrapper = typename T::ConstPtr;

  using Rate = ros::Rate;
  using RosTime = ros::Time;

  template <typename T>
  T* getServicePointer(T& service) { return &service; }
  
#elif ONTO_ROS_VERSION == 2
  template <typename T>
  using ServiceWrapper = typename T::SharedPtr; //std::shared_ptr<T>;

  template <typename T>
  using MessageWrapper = typename T::ConstSharedPtr;

  using Rate = rclcpp::Rate;
  using RosTime = rclcpp::Time;

  using namespace ::ontologenius::msg;
  using namespace ::ontologenius::srv;

  template <typename T>
  T& getServicePointer(T& service) { return service; }
#endif

template <typename T>
class Publisher;

template <typename T>
class Subscriber;

template <typename T>
class Service;

template <typename T>
class Client;

class Time : public RosTime
{
public:
  Time(uint32_t sec, uint32_t nsec) : RosTime(sec, nsec) {}
  explicit Time(double t) : RosTime(t) {}
  Time(const RosTime& time) : RosTime(time) {} // do not put it as explicit

  uint32_t seconds() const
  {
#if ONTO_ROS_VERSION == 1
    return sec;
#elif ONTO_ROS_VERSION == 2
    return RosTime::seconds();
#endif 
  }

  uint32_t nanoseconds() const
  {
#if ONTO_ROS_VERSION == 1
    return nsec;
#elif ONTO_ROS_VERSION == 2
    return RosTime::nanoseconds();
#endif 
  }
};

class Node
{
public:
  template <typename T>
  friend class Publisher;

  template <typename T>
  friend class Subscriber;

  template <typename T>
  friend class Service;

  template <typename T>
  friend class Client;

  Node(Node& other) = delete;
  Node(Node&& other) = delete;
  ~Node() {}

  static Node& get();
  static bool ok();

  static void init(int argc, char **argv, const std::string& node_name);
  static void shutdown();

  void spin();

  Time current_time();

private:
  explicit Node(const std::string& node_name);

  const std::string name_;

#if ONTO_ROS_VERSION == 1
  ros::NodeHandle handle_;
  ros::CallbackQueue callback_queue_;
#elif ONTO_ROS_VERSION == 2
  rclcpp::Node::SharedPtr handle_;
  std::thread ros_thread_;
#endif

  bool running_;
};

template <typename T>
class Publisher
{
public:
  Publisher(const std::string& topic_name, std::size_t queue_size)
  {  
    auto& node = Node::get();

#if ONTO_ROS_VERSION == 1
    handle_ = node.handle_.advertise<T>(topic_name, queue_size);
#elif ONTO_ROS_VERSION == 2
    (void) queue_size;
    handle_ = node.handle_->create_publisher<T>(topic_name, 10);
#endif
  }

  void publish(const T& message)
  {
#if ONTO_ROS_VERSION == 1
    handle_.publish(message);
#elif ONTO_ROS_VERSION == 2
    handle_->publish(message);
#endif
  }

  size_t getNumSubscribers()
  {
#if ONTO_ROS_VERSION == 1
    return handle_.getNumSubscribers();
#elif ONTO_ROS_VERSION == 2
    return handle_->get_subscription_count();
#endif
  }

private:
#if ONTO_ROS_VERSION == 1
  ros::Publisher handle_;
#elif ONTO_ROS_VERSION == 2
  typename rclcpp::Publisher<T>::SharedPtr handle_;
#endif
};

template <typename T>
class Subscriber
{
public:
  template<typename Ta, typename Tb>
  Subscriber(const std::string& topic_name, std::size_t queue_size, Ta&& callback, Tb&& ptr)
  {    
    auto& node = Node::get();
    
#if ONTO_ROS_VERSION == 1
    handle_ = node.handle_.subscribe(topic_name, queue_size, callback, ptr);
#elif ONTO_ROS_VERSION == 2
    (void) queue_size;
    handle_ = node.handle_->create_subscription<T>(topic_name, 10, std::bind(std::forward<Ta>(callback), ptr, std::placeholders::_1));
#endif
  }
  
private:
#if ONTO_ROS_VERSION == 1
  ros::Subscriber handle_;
#elif ONTO_ROS_VERSION == 2
  typename rclcpp::Subscription<T>::SharedPtr handle_;
#endif
};

template <typename T>
class Service
{
public:
  template<typename Ta>
  Service(const std::string& service_name, Ta&& callback)
  {
    auto& node = Node::get();

#if ONTO_ROS_VERSION == 1
    handle_ = node.handle_.advertiseService(service_name, callback);
#elif ONTO_ROS_VERSION == 2
    handle_ = node.handle_->create_service<T>(service_name, [&](compat::onto_ros::ServiceWrapper<typename T::Request> req, compat::onto_ros::ServiceWrapper<typename T::Response> res){ callback(req, res); });
    //handle_ = node.handle_->create_service<T>(service_name, callback);
#endif
  }

  template<typename Ta, typename Tb>
  Service(const std::string& service_name, Ta&& callback, Tb&& ptr)
  {
    auto& node = Node::get();

#if ONTO_ROS_VERSION == 1
    handle_ = node.handle_.advertiseService(service_name, callback, ptr);
#elif ONTO_ROS_VERSION == 2
    handle_ = node.handle_->create_service<T>(service_name, [ptr, callback](compat::onto_ros::ServiceWrapper<typename T::Request> req, compat::onto_ros::ServiceWrapper<typename T::Response> res){ (ptr->*callback)(req, res); });
    //handle_ = node.handle_->create_service<T>(service_name, std::bind(std::forward<Ta>(callback), ptr, std::placeholders::_1, std::placeholders::_2));
#endif
  }
  
private:
#if ONTO_ROS_VERSION == 1
  ros::ServiceServer handle_;
#elif ONTO_ROS_VERSION == 2
  typename rclcpp::Service<T>::SharedPtr handle_;
#endif
};

template <typename T>
class Client
{
public:
  enum class Status { SUCCESSFUL, SUCCESSFUL_WITH_RETRIES, FAILURE };

  explicit Client(const std::string& service_name) : name_(service_name)
  {
    auto& node = Node::get();

#if ONTO_ROS_VERSION == 1
    handle_ = node.handle_.serviceClient<T>(service_name, true);
#elif ONTO_ROS_VERSION == 2
    handle_ = node.handle_->create_client<T>(service_name);
#endif
  }

  Status call(const ontologenius::compat::RequestType<T>& req, ontologenius::compat::ResponseType<T>& res)
  {
    using namespace std::chrono_literals;
    auto status = Status::FAILURE;

#if ONTO_ROS_VERSION == 1
    T srv;
    srv.request = req;
    if (!handle_.call(srv))
    {
      auto& node = Node::get();
      handle_ = node.handle_.serviceClient<T>(name_, true);
      if (handle_.call(srv))
      {
        status = Status::SUCCESSFUL_WITH_RETRIES;
        res = srv.response;
      }
    }
    else
    {
      status = Status::SUCCESSFUL;
      res = srv.response;
    }

#elif ONTO_ROS_VERSION == 2
    if (!handle_->wait_for_service(5s))
    {
      return status;
    }

    auto future = handle_->async_send_request(req);

    if (future.wait_for(5s) == std::future_status::ready)
    {
      status = Status::SUCCESSFUL;
      res = future.get();
    }
#endif
    return status;
  }

  bool wait(double timeout)
  {
#if ONTO_ROS_VERSION == 1
    return handle_.waitForExistence(ros::Duration(timeout));
#elif ONTO_ROS_VERSION == 2
    return handle_->wait_for_service(std::chrono::duration<double>(timeout));
#endif
  }

private:
  std::string name_;
#if ONTO_ROS_VERSION == 1
  ros::ServiceClient handle_;
#elif ONTO_ROS_VERSION == 2
  typename rclcpp::Client<T>::SharedPtr handle_;
#endif
};
    
} // namespace onto_ros

} // namespace ontologenius::compat

#endif // COMPAT_ROS_H
