#ifndef ONTOLOGENIUS_FEEDERECHO_H
#define ONTOLOGENIUS_FEEDERECHO_H

#include <ros/ros.h>
#include "ontologenius/StampedString.h"

namespace ontologenius {

class FeederEcho
{
public:
  FeederEcho(const std::string& topic_name) : feeder_echo_pub_(n_.advertise<ontologenius::StampedString>(topic_name, 1000)) {}

  void add(const std::string& data, const ros::Time& stamp = ros::Time::now())
  {
    messages.emplace_back(data, stamp);
  }

  void publish()
  {
    ontologenius::StampedString ros_msg;
    for(auto& message : messages)
    {
      ros_msg.data = message.first;
      ros_msg.stamp = message.second;
      feeder_echo_pub_.publish(ros_msg);
    }
    messages.clear();
  }

private:
  ros::NodeHandle n_;
  ros::Publisher feeder_echo_pub_;
  std::vector<std::pair<std::string, ros::Time>> messages;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDERECHO_H
