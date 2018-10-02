#include "ontoloGenius/core/arguer/Arguers.h"
#include "ontoloGenius/core/utility/color.h"

#include <iostream>
#include <vector>

Arguers::Arguers(Ontology* onto) : loader_("ontologenius", "ArguerInterface")
{
  ontology_ = onto;
}

Arguers::~Arguers()
{
  for(auto& it : arguers_)
  {
    if(it.second != nullptr)
    {
      delete it.second;
      it.second = nullptr;
    }
    //TODO: unload the library cause segfault or exception => unstable behavior
    try
    {
      loader_.unloadLibraryForClass(it.first);
      std::cout << it.first << " unloaded" << std::endl;
    }
    catch(class_loader::LibraryUnloadException& ex)
    {
      std::cout << "class_loader::LibraryUnloadException on " << it.first << " : " << std::string(ex.what()) << std::endl;
      ROS_ERROR("The plugin %s failed to unload for some reason. Error: %s", it.first.c_str(), ex.what());
    }
    catch(pluginlib::LibraryUnloadException& ex)
    {
      std::cout << "pluginlib::LibraryUnloadException on " << it.first << " : " << std::string(ex.what()) << std::endl;
      ROS_ERROR("The plugin %s failed to unload for some reason. Error: %s", it.first.c_str(), ex.what());
    }
    catch(...)
    {
      std::cout << "catch other" << std::endl;
    }
  }
  for(auto& it : arguers_)
  {
    if(it.second != nullptr)
    {
      std::cout << "delete " << it.first << std::endl;
      delete it.second;
      it.second = nullptr;
    }
  }
}

void Arguers::load()
{
  std::vector<std::string> arguers = loader_.getDeclaredClasses();

  try
  {
    for(size_t i = 0; i < arguers.size(); i++)
    {
      loader_.loadLibraryForClass(arguers[i]);
      ArguerInterface* tmp = loader_.createUnmanagedInstance(arguers[i]);
      tmp->initialize(ontology_);
      arguers_[arguers[i]] = tmp;
      arguers_[arguers[i]] = tmp;
      if(tmp->defaultAvtive())
        active_arguers_[arguers[i]] = tmp;
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  arguers = loader_.getRegisteredLibraries();
  for(size_t i = 0; i < arguers.size(); i++)
  {
    std::cout << arguers[i] << " registered" << std::endl;
  }
}

std::string Arguers::list()
{
  std::string out =  "Plugins loaded :\n";
  std::string res;
  for(auto& it : arguers_)
  {
    out += " - From " + it.first + " : " + arguers_[it.first]->getName() + " (" + arguers_[it.first]->getDesciption() + ")\n";
    res += " -" + it.first;
  }
  return res;
}

std::vector<std::string> Arguers::listVector()
{
  std::string out =  "Plugins loaded :\n";
  std::vector<std::string> res;
  for(auto& it : arguers_)
  {
    out += " - From " + it.first + " : " + arguers_[it.first]->getName() + " (" + arguers_[it.first]->getDesciption() + ")\n";
    res.push_back(it.first);
  }
  return res;
}

int Arguers::activate(std::string plugin)
{
  if(arguers_.find(plugin) != arguers_.end())
  {
    if(active_arguers_.find(plugin) == active_arguers_.end())
    {
      active_arguers_[plugin] = arguers_[plugin];
      std::cout << COLOR_GREEN << plugin << " has been activated" << COLOR_OFF << std::endl;
      runPostArguers();
    }
    else
      std::cout << COLOR_ORANGE << plugin << " is already activated" << COLOR_OFF << std::endl;
    return 0;
  }
  else
  {
    std::cout << COLOR_RED << plugin << " does not exist" << COLOR_OFF << std::endl;
    return -1;
  }
}

int Arguers::deactivate(std::string plugin)
{
  if(active_arguers_.find(plugin) != active_arguers_.end())
  {
    active_arguers_.erase(plugin);
    std::cout << COLOR_GREEN << plugin << " has been deactivated" << COLOR_OFF << std::endl;
    return 0;
  }
  else
  {
    if(arguers_.find(plugin) == arguers_.end())
    {
      std::cout << COLOR_RED << plugin << " does not exist" << COLOR_OFF << std::endl;
      return -1;
    }
    else
    {
      std::cout << COLOR_ORANGE << plugin << " is already deactivated" << COLOR_OFF << std::endl;
      return 0;
    }
  }
}

std::string Arguers::getDescription(std::string plugin)
{
  if(arguers_.find(plugin) != arguers_.end())
    return arguers_[plugin]->getDesciption();
  else
  {
    std::cout << COLOR_RED << plugin << " does not exist" << COLOR_OFF << std::endl;
    return "";
  }
}

void Arguers::runPreArguers()
{
  size_t nb_updates = 0;

  do
  {
    for(auto& it : active_arguers_)
      it.second->preReason();

    computeIndividualsUpdates();

    nb_updates = ArguerInterface::getNbUpdates();
    ArguerInterface::resetNbUpdates();
  }
  while(nb_updates!= 0);
}

void Arguers::runPostArguers()
{
  size_t nb_updates = 0;

  do
  {
    for(auto& it : active_arguers_)
    {
      if(it.second != nullptr)
        it.second->postReason();
    }

    computeIndividualsUpdates();

    nb_updates = ArguerInterface::getNbUpdates();
    ArguerInterface::resetNbUpdates();
  }
  while(nb_updates!= 0);
}

void Arguers::computeIndividualsUpdates()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
    if(indiv[indiv_i]->nb_updates_ == 0)
      indiv[indiv_i]->updated_ = false;
    else
    {
      indiv[indiv_i]->nb_updates_ = 0;
      indiv[indiv_i]->updated_ = true;
    }
}
