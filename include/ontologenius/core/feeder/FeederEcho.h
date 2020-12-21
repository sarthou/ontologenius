#ifndef ONTOLOGENIUS_FEEDERECHO_H
#define ONTOLOGENIUS_FEEDERECHO_H

#include <ros/ros.h>
#include "ontologenius/StampedString.h"
#include "ontologenius/OntologeniusExplanation.h"

namespace ontologenius {

class FeederEcho
{
public:
  FeederEcho(const std::string& echo_topic, const std::string& expl_topic) : feeder_echo_pub_(n_.advertise<ontologenius::StampedString>(echo_topic, 1000)),
                                                                             feeder_expl_pub_(n_.advertise<ontologenius::OntologeniusExplanation>(expl_topic, 1000))
  {}

  void add(const std::string& data, const ros::Time& stamp = ros::Time::now())
  {
    echo_messages.emplace_back(data, stamp);
  }

  void add(const std::vector<std::pair<std::string, std::string>>& explanations)
  {
    expl_messages.insert(expl_messages.end(), explanations.begin(), explanations.end());
  }

  void publish()
  {
    ontologenius::StampedString ros_msg;
    for(auto& message : echo_messages)
    {
      ros_msg.data = message.first;
      ros_msg.stamp = message.second;
      feeder_echo_pub_.publish(ros_msg);
    }
    echo_messages.clear();

    ontologenius::OntologeniusExplanation ros_expl_msg;
    for(auto& message : expl_messages)
    {
      ros_expl_msg.fact = message.first;
      ros_expl_msg.cause = message.second;
      feeder_expl_pub_.publish(ros_expl_msg);
    }
    expl_messages.clear();
  }

private:
  ros::NodeHandle n_;
  ros::Publisher feeder_echo_pub_;
  ros::Publisher feeder_expl_pub_;
  std::vector<std::pair<std::string, ros::Time>> echo_messages;
  std::vector<std::pair<std::string, std::string>> expl_messages;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDERECHO_H
