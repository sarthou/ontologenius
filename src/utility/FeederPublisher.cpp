#include "ontoloGenius/utility/FeederPublisher.h"

void FeederPublisher::addProperty(const std::string& from, const std::string& property, const std::string& on)
{
  std::string msg = "[add]" + from + "|" + property + "|" + on;
  publish(msg);
}

void FeederPublisher::addProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value)
{
  std::string msg = "[add]" + from + "|" + property + "|" + type + ":" + value;
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

void FeederPublisher::removeProperty(const std::string& from, const std::string& property, const std::string& on)
{
  std::string msg = "[del]" + from + "|" + property + "|" + on;
  publish(msg);
}

void FeederPublisher::removeProperty(const std::string& from, const std::string& property, const std::string& type, const std::string& value)
{
  std::string msg = "[del]" + from + "|" + property + "|" + type + ":" + value;
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

void FeederPublisher::publish(std::string& str)
{
  std_msgs::String msg;
  msg.data = str;
  pub_.publish(msg);
}
