#include "ontologenius/API/ontologenius/FeederPublisher.h"

#include <chrono>
#include <unistd.h>

void FeederPublisher::addProperty(const std::string& from, const std::string& property, const std::string& on, const ros::Time& stamp)
{
  std::string msg = "[add]" + from + "|" + property + "|" + on;
  publishStamped(msg, stamp);
}

void FeederPublisher::addProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value, const ros::Time& stamp)
{
  std::string msg = "[add]" + from + "|" + property + "|" + type + "#" + value;
  publishStamped(msg, stamp);
}

void FeederPublisher::addInheritage(const std::string& from, const std::string& on, const ros::Time& stamp)
{
  std::string msg = "[add]" + from + "|+|" + on;
  publishStamped(msg, stamp);
}

void FeederPublisher::addLanguage(const std::string& from, const std::string& lang, const std::string& name, const ros::Time& stamp)
{
  std::string msg = "[add]" + from + "|@" + lang + "|" + name;
  publishStamped(msg, stamp);
}

void FeederPublisher::addConcept(const std::string& from, const ros::Time& stamp)
{
  std::string msg = "[add]" + from + "|";
  publishStamped(msg, stamp);
}

void FeederPublisher::addInverseOf(const std::string& property, const std::string& inverse_property, const ros::Time& stamp)
{
  std::string msg = "[add]" + property + "|<-|" + inverse_property;
  publishStamped(msg, stamp);
}

void FeederPublisher::removeProperty(const std::string& from, const std::string& property, const ros::Time& stamp)
{
  std::string msg = "[del]" + from + "|" + property + "|_";
  publishStamped(msg, ros::Time::now());
  msg += ":_";
  publishStamped(msg, stamp);
}

void FeederPublisher::removeProperty(const std::string& from, const std::string& property, const std::string& on, const ros::Time& stamp)
{
  std::string msg = "[del]" + from + "|" + property + "|" + on;
  publishStamped(msg, stamp);
}

void FeederPublisher::removeProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value, const ros::Time& stamp)
{
  std::string msg = "[del]" + from + "|" + property + "|" + type + "#" + value;
  publishStamped(msg, stamp);
}

void FeederPublisher::removeInheritage(const std::string& from, const std::string& on, const ros::Time& stamp)
{
  std::string msg = "[del]" + from + "|+|" + on;
  publishStamped(msg, stamp);
}

void FeederPublisher::removeLanguage(const std::string& from, const std::string& lang, const std::string& name, const ros::Time& stamp)
{
  std::string msg = "[del]" + from + "|@" + lang + "|" + name;
  publishStamped(msg, stamp);
}

void FeederPublisher::removeConcept(const std::string& from, const ros::Time& stamp)
{
  std::string msg = "[del]" + from + "|";
  publishStamped(msg, stamp);
}

void FeederPublisher::removeInverseOf(const std::string& property, const std::string& inverse_property, const ros::Time& stamp)
{
  std::string msg = "[del]" + property + "|<-|" + inverse_property;
  publishStamped(msg, stamp);
}

bool FeederPublisher::waitUpdate(int32_t timeout)
{
  updated_ = false;

  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  sendNop();

  while((!updated_) && (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-start).count()) < (unsigned int)timeout)
  {
    ros::spinOnce();
    usleep(1);
  }

  if(updated_)
    return true;
  else
    return false;
}

std::string FeederPublisher::commit(int32_t timeout)
{
  std::string commit_name = std::to_string(commit_nb_);
  commit_nb_++;

  if(commit(commit_name, timeout))
    return commit_name;
  else
    return "";
}

bool FeederPublisher::commit(const std::string& commit_name, int32_t timeout)
{
  std::string msg = "[commit]" + commit_name + "|";
  updated_ = false;

  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  publishStamped(msg, ros::Time::now());

  while((!updated_) && (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-start).count()) < (unsigned int)timeout)
  {
    ros::spinOnce();
    usleep(1);
  }

  if(updated_)
    return true;
  else
    return false;
}

bool FeederPublisher::checkout(const std::string& commit_name, int32_t timeout)
{
  std::string msg = "[checkout]" + commit_name + "|";

  updated_ = false;

  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  publishStamped(msg, ros::Time::now());

  while((!updated_) && (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-start).count()) < (unsigned int)timeout)
  {
    ros::spinOnce();
    usleep(1);
  }

  if(updated_)
    return true;
  else
    return false;
}

void FeederPublisher::sendNop()
{
  publishStamped("[nop]nop|", ros::Time::now());
}

void FeederPublisher::publish(const std::string& str)
{
  std_msgs::String msg;
  msg.data = str;
  pub_.publish(msg);
}

void FeederPublisher::publishStamped(const std::string& str, const ros::Time& stamp)
{
  ontologenius::StampedString msg;
  msg.data = str;
  msg.stamp = stamp;
  stamped_pub_.publish(msg);
}

void FeederPublisher::commitCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "end")
    updated_ = true;
}
