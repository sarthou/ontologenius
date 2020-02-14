#include "ontologenius/API/ontologenius/FeederPublisher.h"

#include <chrono>
#include <unistd.h>

void FeederPublisher::addProperty(const std::string& from, const std::string& property, const std::string& on)
{
  std::string msg = "[add]" + from + "|" + property + "|" + on;
  publish(msg);
}

void FeederPublisher::addProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value)
{
  std::string msg = "[add]" + from + "|" + property + "|" + type + "#" + value;
  publish(msg);
}

void FeederPublisher::addInheritage(const std::string& from, const std::string& on)
{
  std::string msg = "[add]" + from + "|+|" + on;
  publish(msg);
}

void FeederPublisher::addLanguage(const std::string& from, const std::string& lang, const std::string& name)
{
  std::string msg = "[add]" + from + "|@" + lang + "|" + name;
  publish(msg);
}

void FeederPublisher::addConcept(const std::string& from)
{
  std::string msg = "[add]" + from + "|";
  publish(msg);
}

void FeederPublisher::removeProperty(const std::string& from, const std::string& property)
{
  std::string msg = "[del]" + from + "|" + property + "|_";
  publish(msg);
  msg += ":_";
  publish(msg);
}

void FeederPublisher::removeProperty(const std::string& from, const std::string& property, const std::string& on)
{
  std::string msg = "[del]" + from + "|" + property + "|" + on;
  publish(msg);
}

void FeederPublisher::removeProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value)
{
  std::string msg = "[del]" + from + "|" + property + "|" + type + "#" + value;
  publish(msg);
}

void FeederPublisher::removeInheritage(const std::string& from, const std::string& on)
{
  std::string msg = "[del]" + from + "|+|" + on;
  publish(msg);
}

void FeederPublisher::removeLanguage(const std::string& from, const std::string& lang, const std::string& name)
{
  std::string msg = "[del]" + from + "|@" + lang + "|" + name;
  publish(msg);
}

void FeederPublisher::removeConcept(const std::string& from)
{
  std::string msg = "[del]" + from + "|";
  publish(msg);
}

bool FeederPublisher::commit(int32_t timeout)
{
  commited_ = false;
  commit_sub_ = n_->subscribe(name_ == "" ? "ontologenius/end" : "ontologenius/end/" + name_, 1000, &FeederPublisher::commitCallback, this);

  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();

  while((!commited_) && (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-start).count()) < (unsigned int)timeout)
  {
    ros::spinOnce();
    usleep(1);
  }

  commit_sub_.shutdown();
  if(commited_)
    return true;
  else
    return false;
}

void FeederPublisher::publish(std::string& str)
{
  std_msgs::String msg;
  msg.data = str;
  pub_.publish(msg);
}

void FeederPublisher::commitCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "end")
    commited_ = true;
}
