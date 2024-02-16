#ifndef ONTOLOGENIUS_FEEDERECHO_H
#define ONTOLOGENIUS_FEEDERECHO_H

#include <mutex>

#include "ontologenius/compat/ros.h"

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
    echo_messages.clear();
    expl_messages.clear();
    // should we perhaps unlock the mutex...?
  }

  void add(const std::string& data, const compat::onto_ros::Time& stamp = compat::onto_ros::Node::get().current_time())
  {
    mut_.lock();
    echo_messages.emplace_back(data, stamp);
    mut_.unlock();
  }

  void add(const std::vector<std::pair<std::string, RosTime_t>>& facts)
  {
    mut_.lock();
    for(auto& fact : facts)
      echo_messages.emplace_back(fact.first, compat::onto_ros::Time { fact.second.sec, fact.second.nsec } );
    mut_.unlock();
  }

  void add(const std::vector<std::pair<std::string, std::string>>& explanations)
  {
    mut_.lock();
    expl_messages.insert(expl_messages.end(), explanations.begin(), explanations.end());
    mut_.unlock();
  }

  void publish()
  {
    mut_.lock();
    ontologenius::compat::OntologeniusStampedString ros_msg;
    for(auto& message : echo_messages)
    {
      ros_msg.data = message.first;
      ros_msg.stamp.seconds = message.second.seconds();
      ros_msg.stamp.nanoseconds = message.second.nanoseconds();
      feeder_echo_pub_.publish(ros_msg);
    }
    echo_messages.clear();

    ontologenius::compat::OntologeniusExplanation ros_explanation_msg;
    for(auto& message : expl_messages)
    {
      ros_explanation_msg.fact = message.first;
      ros_explanation_msg.cause = message.second;
      feeder_explanation_pub_.publish(ros_explanation_msg);
    }
    expl_messages.clear();
    mut_.unlock();
  }

private:
  std::mutex mut_;
  compat::onto_ros::Publisher<compat::OntologeniusStampedString> feeder_echo_pub_;
  compat::onto_ros::Publisher<compat::OntologeniusExplanation> feeder_explanation_pub_;

  std::vector<std::pair<std::string, compat::onto_ros::Time>> echo_messages;
  std::vector<std::pair<std::string, std::string>> expl_messages;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDERECHO_H
