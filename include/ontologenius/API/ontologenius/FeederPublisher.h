#ifndef ONTOLOGENIUS_FEEDERPUBLISHER_H
#define ONTOLOGENIUS_FEEDERPUBLISHER_H

#include <atomic>
#include <stdlib.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ontologenius/StampedString.h"

class FeederPublisher
{
public:
  FeederPublisher(ros::NodeHandle* n, const std::string& name) :
                  pub_(n->advertise<std_msgs::String>((name == "") ? "ontologenius/insert" : "ontologenius/insert/" + name, 1000)),
                  stamped_pub_(n->advertise<ontologenius::StampedString>((name == "") ? "ontologenius/insert_stamped" : "ontologenius/insert_stamped/" + name, 1000))
  {
    n_ = n;
    name_ = name;
    srand (time(NULL));
    commit_nb_ = rand() % 100000 + 1;
    commit_sub_ = n_->subscribe(name_ == "" ? "ontologenius/end" : "ontologenius/end/" + name_, 1000, &FeederPublisher::commitCallback, this);
    updated_ = false;
  }

  FeederPublisher(FeederPublisher& other) :
                  pub_(other.n_->advertise<std_msgs::String>((other.name_ == "") ? "ontologenius/insert" : "ontologenius/insert/" + other.name_, 1000)),
                  stamped_pub_(other.n_->advertise<ontologenius::StampedString>((other.name_ == "") ? "ontologenius/insert_stamped" : "ontologenius/insert_stamped/" + other.name_, 1000))
  {
    n_ = other.n_;
    name_ = other.name_;
    commit_sub_ = n_->subscribe(name_ == "" ? "ontologenius/end" : "ontologenius/end/" + name_, 1000, &FeederPublisher::commitCallback, this);
    commit_nb_ = other.commit_nb_;
    updated_ = false;
  }

  void addProperty(const std::string& from, const std::string& property, const std::string& on);
  void addProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value);
  void addInheritage(const std::string& from, const std::string& on);
  void addLanguage(const std::string& from, const std::string& lang, const std::string& name);
  void addConcept(const std::string& from);

  void removeProperty(const std::string& from, const std::string& property);
  void removeProperty(const std::string& from, const std::string& property, const std::string& on);
  void removeProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value);
  void removeInheritage(const std::string& from, const std::string& on);
  void removeLanguage(const std::string& from, const std::string& lang, const std::string& name);
  void removeConcept(const std::string& from);

  size_t getNumSubscribers() { return stamped_pub_.getNumSubscribers(); }
  void waitConnected()
  {
    ros::Rate loop_rate(100);
    while(ros::ok() && (getNumSubscribers() == 0))
      loop_rate.sleep();
  }

  bool waitUpdate(int32_t timeout = -1);
  std::string commit(int32_t timeout = -1);
  bool commit(const std::string& commit_name, int32_t timeout = -1);
  bool checkout(const std::string& commit_name, int32_t timeout = -1);

private:
  ros::NodeHandle* n_;
  std::string name_;
  ros::Publisher pub_;
  ros::Publisher stamped_pub_;
  ros::Subscriber commit_sub_;
  std::atomic<bool> updated_;
  size_t commit_nb_;

  void sendNop();
  void publish(const std::string& str);
  void publishStamped(const std::string& str, const ros::Time& stamp);

  void commitCallback(const std_msgs::String::ConstPtr& msg);
};

#endif // ONTOLOGENIUS_FEEDERPUBLISHER_H
