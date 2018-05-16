#include "ontoloGenius/arguer/Arguers.h"

#include <iostream>
#include <vector>

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE  "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN   "\x1B[1;92m"
#endif

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
      if(tmp->defaultAvtive())
        active_arguers_[arguers[i]] = tmp;

    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
}

std::string Arguers::list()
{
  std::map<std::string, ArguerInterface*>::iterator it;
  std::string out =  "Plugins loaded :\n";
  std::string res;
  for(it = arguers_.begin(); it != arguers_.end(); ++it)
  {
    out += " - From " + it->first + " : " + arguers_[it->first]->getName() + " (" + arguers_[it->first]->getDesciption() + ")\n";
    res += " -" + it->first;
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
  std::map<std::string, ArguerInterface*>::iterator it;
  for(it = active_arguers_.begin(); it != active_arguers_.end(); ++it)
    it->second->preReason();
}

void Arguers::runPostArguers()
{
  size_t nb_updates = 0;

  do
  {
    std::map<std::string, ArguerInterface*>::iterator it;
    for(it = active_arguers_.begin(); it != active_arguers_.end(); ++it)
      it->second->postReason();

    computeIndividualsUpdates();

    nb_updates = ArguerInterface::getNbUpdates();
    ArguerInterface::resetNbUpdates();
  }
  while(nb_updates!= 0);
}

void Arguers::computeIndividualsUpdates()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  for(size_t indiv_i = 0; indiv_i < indiv.size(); indiv_i++)
    if(indiv[indiv_i]->nb_updates_ == 0)
      indiv[indiv_i]->updated_ = false;
    else
    {
      indiv[indiv_i]->nb_updates_ = 0;
      indiv[indiv_i]->updated_ = true;
    }
}
