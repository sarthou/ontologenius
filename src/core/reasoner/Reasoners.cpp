#include "ontologenius/core/reasoner/Reasoners.h"

#include <iostream>
#include <vector>

#include "ontologenius/graphical/Display.h"

namespace ontologenius {

Reasoners::Reasoners(const std::string& agent_name, Ontology* onto) : agent_name_(agent_name),
                                                                      loader_("ontologenius", "ontologenius::ReasonerInterface")
{
  ontology_ = onto;
}

Reasoners::~Reasoners()
{
  for(auto& it : reasoners_)
  {
    if(it.second != nullptr)
    {
      delete it.second;
      it.second = nullptr;
    }
  }

  for(auto& it : reasoners_)
  {
    try
    {
      loader_.unloadLibraryForClass(it.first);
      std::cout << it.first << " unloaded" << std::endl;
    }
    catch(class_loader::LibraryUnloadException& ex)
    {
      Display::error("class_loader::LibraryUnloadException on " + it.first + " : " + std::string(ex.what()));
    }
    catch(pluginlib::LibraryUnloadException& ex)
    {
      Display::error("pluginlib::LibraryUnloadException on " + it.first + " : " + std::string(ex.what()));
    }
    catch(...)
    {
      std::cout << "catch other" << std::endl;
    }
  }
}

void Reasoners::link(Ontology* onto)
{
  ontology_ = onto;
  for(auto& it : reasoners_)
    it.second->initialize(agent_name_, ontology_);
}

void Reasoners::configure(const std::string& config_path)
{
  if((config_path != "") && (config_path != "none"))
  {
    if(config_.read(config_path) == false)
      Display::error("Fail to load configuration file: " + config_path);
    else
      Display::success("The reasoners have been configured");
  }
}

void Reasoners::load()
{
  std::vector<std::string> reasoners = loader_.getDeclaredClasses();

  try
  {
    for(auto& reasoner : reasoners)
    {
      loader_.loadLibraryForClass(reasoner);
      ReasonerInterface* tmp = loader_.createUnmanagedInstance(reasoner);
      tmp->initialize(agent_name_, ontology_);
      reasoners_[reasoner] = tmp;
      if(tmp->defaultActive())
        active_reasoners_[reasoner] = tmp;
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    Display::error("The plugin failed to load for some reason. Error: " + std::string(ex.what()));
  }

  reasoners = loader_.getRegisteredLibraries();
  for(auto& reasoner : reasoners)
  {
    std::cout << reasoner << " registered" << std::endl;
  }

  applyConfig();
}

void Reasoners::initialize()
{
  for(auto& it : reasoners_)
    it.second->initialize();
}

std::string Reasoners::list()
{
  std::string res;
  for(auto& it : reasoners_)
    res += " -" + it.first;
  return res;
}

std::vector<std::string> Reasoners::listVector()
{
  std::vector<std::string> res;
  std::transform(reasoners_.cbegin(), reasoners_.cend(), std::back_inserter(res), [](const auto& it){ return it.first; });
  return res;
}

std::vector<std::string> Reasoners::activeListVector()
{
  std::vector<std::string> res;
  std::transform(active_reasoners_.cbegin(), active_reasoners_.cend(), std::back_inserter(res), [](const auto& it){ return it.first; });
  return res;
}

int Reasoners::activate(const std::string& plugin)
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

int Reasoners::deactivate(const std::string& plugin)
{
  if(active_reasoners_.erase(plugin))
  {
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
  {
    std::string description = reasoners_[plugin]->getDescription();
    if(reasoners_[plugin]->implementPostReasoning())
      description += "\n - post reasoning";
    if(reasoners_[plugin]->implementPreReasoning())
      description += "\n - pre reasoning";
    if(reasoners_[plugin]->implementPeriodicReasoning())
      description += "\n - periodic reasoning";
    return description;
  }
  else
  {
    Display::error(plugin + " does not exist");
    return "";
  }
}

void Reasoners::runPreReasoners(QueryOrigin_e origin, const std::string& action, const std::string& param)
{
  size_t nb_updates = 0;

  QueryInfo_t query_info = extractQueryInfo(origin, action, param);

  for(auto& it : active_reasoners_)
  {
    if(it.second)
    {
      if(it.second->preReason(query_info))
      {
        auto notif = it.second->getNotifications();
        notifications_.insert(notifications_.end(), notif.begin(), notif.end());
        auto explanations = it.second->getExplanations();
        explanations_mutex_.lock();
        explanations_.insert(explanations_.end(), explanations.begin(), explanations.end());
        explanations_mutex_.unlock();
      }
    }
  }

  nb_updates = ReasonerInterface::getNbUpdates();
  ReasonerInterface::resetNbUpdates();

  if(nb_updates!= 0)
  {
    computeUpdates();

    runPostReasoners();
  }
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
        auto notif = it.second->getNotifications();
        notifications_.insert(notifications_.end(), notif.begin(), notif.end());
        auto explanations = it.second->getExplanations();
        explanations_mutex_.lock();
        explanations_.insert(explanations_.end(), explanations.begin(), explanations.end());
        explanations_mutex_.unlock();
      }
    }
    nb_updates = ReasonerInterface::getNbUpdates();
    ReasonerInterface::resetNbUpdates();

    computeUpdates();
  }
  while(nb_updates!= 0);
}

void Reasoners::runPeriodicReasoners()
{
  bool has_run = false;

  for(auto& it : active_reasoners_)
  {
    if(it.second != nullptr)
    {
      bool this_has_run = it.second->periodicReason();
      if(this_has_run)
      {
        has_run = true; 
        auto notif = it.second->getNotifications();
        notifications_.insert(notifications_.end(), notif.begin(), notif.end());
        auto explanations = it.second->getExplanations();
        explanations_mutex_.lock();
        explanations_.insert(explanations_.end(), explanations.begin(), explanations.end());
        explanations_mutex_.unlock();
      }
    }
  }

  if(has_run)
    computeIndividualsUpdatesPeriodic();
}

void Reasoners::applyConfig()
{
  for(auto& param : config_.config_)
  {
    if(param.first == "flow")
    {
      if(param.second.data)
      {
        active_reasoners_.clear();
        for(auto& elem : param.second.data.value())
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
          for(auto reasoner_param : elem.second.data.value())
            reasoners_[param.first]->setParameter(elem.first, reasoner_param);
      }
    }
    else
      Display::warning("[CONFIG] no reasoner named " + param.first + ". This parameter will be ignored");
  }
}

void Reasoners::computeUpdates()
{
  computeGraphUpdates(&ontology_->individual_graph_);
  computeGraphUpdates(&ontology_->class_graph_);
  computeGraphUpdates(&ontology_->object_property_graph_);
  computeGraphUpdates(&ontology_->data_property_graph_);
}

template<typename B>
void Reasoners::computeGraphUpdates(Graph<B>* graph)
{
  std::vector<B*> branches = graph->get();
  for(auto& branch : branches)
    if(branch->nb_updates_ == 0)
      branch->updated_ = false;
    else
    {
      branch->nb_updates_ = 0;
      branch->updated_ = true;
    }
}

void Reasoners::computeIndividualsUpdatesPeriodic()
{
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.getSafe();
  for(auto& indiv : indivs)
    if(indiv->nb_updates_ != 0)
    {
      indiv->nb_updates_ = 0;
      indiv->updated_ = true;
    }
}

void Reasoners::resetIndividualsUpdates()
{
  std::vector<IndividualBranch_t*> indivs = ontology_->individual_graph_.get();
  for(auto& indiv : indivs)
  {
    indiv->nb_updates_ = 0;
    indiv->updated_ = true;
  }
}

QueryInfo_t Reasoners::extractQueryInfo(QueryOrigin_e origin, const std::string& action, const std::string& param)
{
  QueryInfo_t query_info;
  query_info.query_origin = origin;

  if((action == "getUp") || (action == "isA"))
  {
    query_info.query_type = query_inheritance;
    query_info.subject = param;
  }
  else if((action == "getDown") || (action == "getType"))
  {
    query_info.query_type = query_inheritance;
    query_info.object = param;
  }
  else if((action == "getRelationOn") || (action == "getRelatedWith"))
  {
    query_info.query_type = query_relation;
    query_info.object = param;
  }
  else if((action == "getRelationFrom") || (action == "getRelationWith"))
  {
    query_info.query_type = query_relation;
    query_info.subject = param;
  }
  else if((action == "getRelatedFrom") || (action == "getRelatedOn"))
  {
    query_info.query_type = query_relation;
    query_info.predicate = param;
  }
  else if(action == "getOn")
  {
    query_info.query_type = query_relation;
    size_t pose = param.find(':');
    if(pose != std::string::npos)
    {
      query_info.subject = param.substr(0, pose);
      query_info.predicate = param.substr(pose+1);
    }
  }
  else if(action == "getFrom")
  {
    query_info.query_type = query_relation;
    size_t pose = param.find(':');
    if(pose != std::string::npos)
    {
      query_info.object = param.substr(0, pose);
      query_info.predicate = param.substr(pose+1);
    }
  }
  else if(action == "getWith")
  {
    query_info.query_type = query_relation;
    size_t pose = param.find(':');
    if(pose != std::string::npos)
    {
      query_info.subject = param.substr(0, pose);
      query_info.object = param.substr(pose+1);
    }
  }
  else if((action == "getName") || (action == "getNames") ||
          (action == "getEveryNames") || (action == "find") ||
          (action == "findSub") || (action == "findRegex") ||
          (action == "findFuzzy"))
  {
    query_info.query_type = query_label;
    query_info.subject = param;
  }
  else
  {
    query_info.query_type = query_other;
    query_info.subject = param;
  }

  return query_info;
}

} // namespace ontologenius
