#include "ontoloGenius/arguer/Arguers.h"

#include <iostream>
#include <vector>

Arguers::~Arguers()
{
  std::map<std::string, ArguerInterface*>::iterator it;
  for(it = arguers_.begin(); it != arguers_.end(); ++it)
  {
    delete it->second;
    loader_.unloadLibraryForClass(it->first);
  }
}

void Arguers::load()
{
  std::vector<std::string> arguers = loader_.getDeclaredClasses();

  try
  {
    for(size_t i = 0; i < arguers.size(); i++)
    {
      ArguerInterface* tmp = loader_.createUnmanagedInstance(arguers[i]);
      tmp->initialize(ontology_);
      arguers_[arguers[i]] = tmp;
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
}

void Arguers::list()
{
  std::map<std::string, ArguerInterface*>::iterator it;
  std::cout << "Pugins loaded :" << std::endl;
  for(it = arguers_.begin(); it != arguers_.end(); ++it)
    std::cout << " - From " << it->first << " : " << arguers_[it->first]->getName() << std::endl;
}

int Arguers::activate(std::string plugin)
{
  if(arguers_.find(plugin) != arguers_.end())
  {
    if(active_arguers_.find(plugin) == active_arguers_.end())
      active_arguers_[plugin] = arguers_[plugin];

    return 0;
  }
  else
    return -1;
}

int Arguers::deactivate(std::string plugin)
{
  if(active_arguers_.find(plugin) != active_arguers_.end())
  {
    active_arguers_.erase(plugin);
    return 0;
  }
  else
    return -1;
}

void Arguers::runPreArguers()
{
  std::map<std::string, ArguerInterface*>::iterator it;
  for(it = arguers_.begin(); it != arguers_.end(); ++it)
    it->second->preReason();
}

void Arguers::runPostArguers()
{
  std::map<std::string, ArguerInterface*>::iterator it;
  for(it = arguers_.begin(); it != arguers_.end(); ++it)
    it->second->postReason();
}
