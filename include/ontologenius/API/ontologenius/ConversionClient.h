#ifndef ONTOLOGENIUS_CONVERTIONCLIENT_H
#define ONTOLOGENIUS_CONVERTIONCLIENT_H

#include <string>

#if ROS_VERSION == 1
#include <ros/ros.h>
#include <ontologenius/OntologeniusConversion.h>
#elif ROS_VERSION == 2
#include <rclcpp/rclcpp.hpp>
#include <ontologenius/srv/OntologeniusConversion.h>
#endif

namespace onto {

class ConversionClient
{
public:
  /// @brief Constructs a ROS conversion client.
  /// @param name is the name of the ontologenius service
  explicit ConversionClient(const std::string& name);

  void verbose(bool verbose) { verbose_ = verbose; }

  std::vector<std::string> individualsIndex2Id(const std::vector<int64_t>& indexes);
  std::vector<std::string> classesIndex2Id(const std::vector<int64_t>& indexes);
  std::vector<std::string> dataPropertiesIndex2Id(const std::vector<int64_t>& indexes);
  std::vector<std::string> objectPropertiesIndex2Id(const std::vector<int64_t>& indexes);
  std::vector<std::string> literalsIndex2Id(const std::vector<int64_t>& indexes);

  std::string individualsIndex2Id(int64_t index);
  std::string classesIndex2Id(int64_t index);
  std::string dataPropertiesIndex2Id(int64_t index);
  std::string objectPropertiesIndex2Id(int64_t index);
  std::string literalsIndex2Id(int64_t index);

  std::vector<int64_t> individualsId2Index(const std::vector<std::string>& ids);
  std::vector<int64_t> classesId2Index(const std::vector<std::string>& ids);
  std::vector<int64_t> dataPropertiesId2Index(const std::vector<std::string>& ids);
  std::vector<int64_t> objectPropertiesId2Index(const std::vector<std::string>& ids);
  std::vector<int64_t> literalsId2Index(const std::vector<std::string>& ids);

  int64_t individualsId2Index(const std::string& id);
  int64_t classesId2Index(const std::string& id);
  int64_t dataPropertiesId2Index(const std::string& id);
  int64_t objectPropertiesId2Index(const std::string& id);
  int64_t literalsId2Index(const std::string& id);

private:
  std::string name_;
  ros::NodeHandle n_;
  bool verbose_;
  ros::ServiceClient client_;

  std::vector<std::string> index2Id(const std::vector<int64_t>& indexes, int8_t source);
  std::string index2Id(int64_t index, int8_t source);

  std::vector<int64_t> Id2Index(const std::vector<std::string>& ids, int8_t source);
  int64_t Id2Index(const std::string& id, int8_t source);

  inline bool call(ontologenius::OntologeniusConversion& srv);
};

} // namespace onto

#endif // ONTOLOGENIUS_CONVERTIONCLIENT_H