#include "ontologenius/API/ontologenius/OntologiesManipulator.h"
#include "ontologenius/graphical/Display.h"

OntologiesManipulator::OntologiesManipulator(ros::NodeHandle* n) : ManagerClient(&n_)
{
  (void)n;
  ontologenius::Display::warning("OntologiesManipulator(ros::NodeHandle* n) is deprecated. Use OntologiesManipulator() instead.");
}

OntologiesManipulator::OntologiesManipulator() : ManagerClient(&n_)
{}

OntologiesManipulator::~OntologiesManipulator()
{
  for(auto& manipulator : manipulators_)
    if(manipulator.second != nullptr)
      delete manipulator.second;
}

void OntologiesManipulator::waitInit(int32_t timeout)
{
  ros::service::waitForService("ontologenius/manage", timeout);
}

OntologyManipulator* OntologiesManipulator::operator[](const std::string& name)
{
  if(manipulators_.find(name) != manipulators_.end())
    return manipulators_[name];
  else
    return nullptr;
}

OntologyManipulator* OntologiesManipulator::get(const std::string& name)
{
  if(manipulators_.find(name) != manipulators_.end())
    return manipulators_[name];
  else
    return nullptr;
}

bool OntologiesManipulator::add(const std::string& name)
{
  if(manipulators_.find(name) != manipulators_.end())
    return true;
  else
  {
    if(ManagerClient::add(name) == false)
      return false;
    else
    {
      ros::service::waitForService("ontologenius/sparql/" + name);
      manipulators_[name] = new OntologyManipulator(name);
      return true;
    }
  }
}

bool OntologiesManipulator::copy(const std::string& dest_name, const std::string& src_name)
{
  if(manipulators_.find(dest_name) != manipulators_.end())
    return true;
  else
  {
    if(ManagerClient::copy(dest_name, src_name) == false)
      return false;
    else
    {
      auto tmp = new OntologyManipulator(dest_name);
      manipulators_[dest_name] = tmp;
      return true;
    }
  }
}

bool OntologiesManipulator::del(const std::string& name)
{
  if(manipulators_.find(name) == manipulators_.end())
    return true;
  else
  {
    if(ManagerClient::del(name) == false)
      return false;
    else
    {
      delete manipulators_[name];
      manipulators_.erase(name);
      return true;
    }
  }
}
