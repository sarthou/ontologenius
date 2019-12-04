#include "ontoloGenius/utility/OntologiesManipulator.h"

OntologiesManipulator::OntologiesManipulator(ros::NodeHandle* n) : ManagerClient(n)
{
  n_ = n;
}

OntologiesManipulator::~OntologiesManipulator()
{
  for(auto manipulator : manipulators_)
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
      OntologyManipulator* tmp = new OntologyManipulator(n_, name);
      manipulators_[name] = tmp;
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
      OntologyManipulator* tmp = new OntologyManipulator(n_, dest_name);
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
