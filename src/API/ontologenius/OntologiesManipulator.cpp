#include "ontologenius/API/ontologenius/OntologiesManipulator.h"

namespace onto {

OntologiesManipulator::OntologiesManipulator() : ManagerClient()
{}

OntologiesManipulator::~OntologiesManipulator()
{
  for(auto& manipulator : manipulators_)
    if(manipulator.second != nullptr)
      delete manipulator.second;

  for(auto& manipulator : manipulators_index_)
    if(manipulator.second != nullptr)
      delete manipulator.second;
}

bool OntologiesManipulator::waitInit(int32_t timeout)
{
  return client_.wait(timeout);
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

OntologyManipulatorIndex* OntologiesManipulator::getIndex(const std::string& name)
{
  if(manipulators_index_.find(name) != manipulators_index_.end())
    return manipulators_index_[name];
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
      manipulators_[name] = new OntologyManipulator(name);
      manipulators_index_[name] = new OntologyManipulatorIndex(name);
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
      auto tmp_index = new OntologyManipulatorIndex(dest_name);
      manipulators_index_[dest_name] = tmp_index;
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
      delete manipulators_index_[name];
      manipulators_index_.erase(name);
      return true;
    }
  }
}

} // namespace onto