#include "ontoloGenius/core/reasoner/Reasoners.h"

#include <iostream>
#include <vector>

#include "ontoloGenius/graphical/Display.h"

namespace ontologenius {

Reasoners::Reasoners(Ontology* onto) : loader_("ontologenius", "ReasonerInterface")
{
  ontology_ = onto;
}

Reasoners::~Reasoners()
{
  for(auto& it : reasoners_)
  {
    /*if(it.second != nullptr)
    {
      delete it.second;
      it.second = nullptr;
    }*/
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
  /*for(auto& it : reasoners_)
  {
    if(it.second != nullptr)
    {
      std::cout << "delete " << it.first << std::endl;
      delete it.second;
      it.second = nullptr;
    }
  }*/
}

void Reasoners::link(Ontology* onto)
{
  ontology_ = onto;
  for(auto& it : reasoners_)
    it.second->initialize(ontology_);
}

void Reasoners::configure(const std::string& config_path)
{
  if((config_path != "") && (config_path != "none"))
  {
    config_.read(config_path);
  }
}

void Reasoners::load()
{
  std::vector<std::string> reasoners = loader_.getDeclaredClasses();

  try
  {
    for(size_t i = 0; i < reasoners.size(); i++)
    {
      loader_.loadLibraryForClass(reasoners[i]);
      ReasonerInterface* tmp = loader_.createUnmanagedInstance(reasoners[i]);
      tmp->initialize(ontology_);
      reasoners_[reasoners[i]] = tmp;
      if(tmp->defaultAvtive())
        active_reasoners_[reasoners[i]] = tmp;
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  reasoners = loader_.getRegisteredLibraries();
  for(size_t i = 0; i < reasoners.size(); i++)
  {
    std::cout << reasoners[i] << " registered" << std::endl;
  }

  applyConfig();
}

std::string Reasoners::list()
{
  std::string out =  "Plugins loaded :\n";
  std::string res;
  for(auto& it : reasoners_)
  {
    out += " - From " + it.first + " : " + reasoners_[it.first]->getName() + " (" + reasoners_[it.first]->getDesciption() + ")\n";
    res += " -" + it.first;
  }
  return res;
}

std::vector<std::string> Reasoners::listVector()
{
  std::string out =  "Plugins loaded :\n";
  std::vector<std::string> res;
  for(auto& it : reasoners_)
  {
    out += " - From " + it.first + " : " + reasoners_[it.first]->getName() + " (" + reasoners_[it.first]->getDesciption() + ")\n";
    res.push_back(it.first);
  }
  return res;
}

std::vector<std::string> Reasoners::activeListVector()
{
  std::vector<std::string> res;
  for(auto& it : active_reasoners_)
  {
    res.push_back(it.first);
  }
  return res;
}

int Reasoners::activate(std::string plugin)
{
  if(reasoners_.find(plugin) != reasoners_.end())
  {
    if(active_reasoners_.find(plugin) == active_reasoners_.end())
    {
      active_reasoners_[plugin] = reasoners_[plugin];
      Display::success(plugin + " has been activated");
      resetIndividualsUpdates();
      runPostReasoners();
    }
    else
      Display::warning(plugin + " is already activated");
    return 0;
  }
  else
  {
    Display::error(plugin + " does not exist");
    return -1;
  }
}

int Reasoners::deactivate(std::string plugin)
{
  if(active_reasoners_.find(plugin) != active_reasoners_.end())
  {
    active_reasoners_.erase(plugin);
    Display::success(plugin + " has been deactivated");
    return 0;
  }
  else
  {
    if(reasoners_.find(plugin) == reasoners_.end())
    {
      Display::error(plugin + " does not exist");
      return -1;
    }
    else
    {
      Display::warning(plugin + " is already deactivated");
      return 0;
    }
  }
}

std::string Reasoners::getDescription(std::string& plugin)
{
  if(reasoners_.find(plugin) != reasoners_.end())
    return reasoners_[plugin]->getDesciption();
  else
  {
    Display::error(plugin + " does not exist");
    return "";
  }
}

void Reasoners::runPreReasoners()
{
  size_t nb_updates = 0;

  do
  {
    for(auto& it : active_reasoners_)
    {
      it.second->preReason();
      std::vector<std::string> notif = it.second->getNotifications();
      notifications_.insert(notifications_.end(), notif.begin(), notif.end());
    }

    computeIndividualsUpdates();

    nb_updates = ReasonerInterface::getNbUpdates();
    ReasonerInterface::resetNbUpdates();
  }
  while(nb_updates!= 0);
}

void Reasoners::runPostReasoners()
{
  size_t nb_updates = 0;

  do
  {
    for(auto& it : active_reasoners_)
    {
      if(it.second != nullptr)
      {
        it.second->postReason();
        std::vector<std::string> notif = it.second->getNotifications();
        notifications_.insert(notifications_.end(), notif.begin(), notif.end());
      }
    }

    computeIndividualsUpdates();

    nb_updates = ReasonerInterface::getNbUpdates();
    ReasonerInterface::resetNbUpdates();
  }
  while(nb_updates!= 0);
}

void Reasoners::runPeriodicReasoners()
{
  for(auto& it : active_reasoners_)
  {
    if(it.second != nullptr)
    {
      it.second->periodicReason();
      std::vector<std::string> notif = it.second->getNotifications();
      notifications_.insert(notifications_.end(), notif.begin(), notif.end());
    }
  }

  computeIndividualsUpdatesPeriodic();
}

void Reasoners::applyConfig()
{
  for(auto param : config_.config_)
  {
    if(param.first == "flow")
    {
      if(param.second.data)
      {
        active_reasoners_.clear();
        for(auto elem : param.second.data.value())
        {
          if(reasoners_.find(elem) != reasoners_.end())
            active_reasoners_[elem] = (reasoners_[elem]);
          else
            Display::warning("[CONFIG] no reasoner named " + elem + ". This reasoner will be ignored");
        }
      }
    }
    else if(reasoners_.find(param.first) != reasoners_.end())
    {
      for(auto elem : param.second.subelem.value())
      {
        if(elem.second.data && elem.second.data.value().size())
          reasoners_[param.first]->setParameter(elem.first, elem.second.data.value()[0]);
      }
    }
    else
      Display::warning("[CONFIG] no reasoner named " + param.first + ". This parameter will be ignored");
  }
}

void Reasoners::computeIndividualsUpdates()
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

void Reasoners::computeIndividualsUpdatesPeriodic()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.getSafe();
  size_t indiv_size = indiv.size();
  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
    if(indiv[indiv_i]->nb_updates_ != 0)
    {
      indiv[indiv_i]->nb_updates_ = 0;
      indiv[indiv_i]->updated_ = true;
    }
}


void Reasoners::resetIndividualsUpdates()
{
  std::vector<IndividualBranch_t*> indiv = ontology_->individual_graph_.get();
  size_t indiv_size = indiv.size();
  for(size_t indiv_i = 0; indiv_i < indiv_size; indiv_i++)
  {
    indiv[indiv_i]->nb_updates_ = 0;
    indiv[indiv_i]->updated_ = true;
  }
}

} // namespace ontologenius
