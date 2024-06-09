#ifndef ONTOLOGENIUS_FEEDERECHO_H
#define ONTOLOGENIUS_FEEDERECHO_H

#include <mutex>
#include <string>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/feeder/FeedStorage.h"

namespace ontologenius {

  class FeederEcho
  {
  public:
    FeederEcho(const std::string& echo_topic, const std::string& expl_topic) : feeder_echo_pub_(echo_topic, 1000),
                                                                               feeder_explanation_pub_(expl_topic, 1000)
    {}

    ~FeederEcho()
    {
      mut_.lock();
      echo_messages_.clear();
      expl_messages_.clear();
      // should we perhaps unlock the mutex...?
    }

    void add(const std::string& data, const compat::onto_ros::Time& stamp = compat::onto_ros::Node::get().currentTime())
    {
      mut_.lock();
      echo_messages_.emplace_back(data, stamp);
      mut_.unlock();
    }

    void add(const std::vector<std::pair<std::string, RosTime_t>>& facts)
    {
      mut_.lock();
      for(const auto& fact : facts)
        echo_messages_.emplace_back(fact.first, compat::onto_ros::Time{fact.second.sec, fact.second.nsec});
      mut_.unlock();
    }

    void add(const std::vector<std::pair<std::string, std::string>>& explanations)
    {
      mut_.lock();
      expl_messages_.insert(expl_messages_.end(), explanations.begin(), explanations.end());
      mut_.unlock();
    }

    void publish()
    {
      mut_.lock();
      ontologenius::compat::OntologeniusStampedString ros_msg;
      for(auto& message : echo_messages_)
      {
        ros_msg.data = message.first;
        ros_msg.stamp.seconds = message.second.seconds();
        ros_msg.stamp.nanoseconds = message.second.nanoseconds();
        feeder_echo_pub_.publish(ros_msg);
      }
      echo_messages_.clear();

      ontologenius::compat::OntologeniusExplanation ros_explanation_msg;
      for(auto& message : expl_messages_)
      {
        ros_explanation_msg.fact = message.first;
        ros_explanation_msg.cause = message.second;
        feeder_explanation_pub_.publish(ros_explanation_msg);
      }
      expl_messages_.clear();
      mut_.unlock();
    }

  private:
    std::mutex mut_;
    compat::onto_ros::Publisher<compat::OntologeniusStampedString> feeder_echo_pub_;
    compat::onto_ros::Publisher<compat::OntologeniusExplanation> feeder_explanation_pub_;

    std::vector<std::pair<std::string, compat::onto_ros::Time>> echo_messages_;
    std::vector<std::pair<std::string, std::string>> expl_messages_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDERECHO_H
