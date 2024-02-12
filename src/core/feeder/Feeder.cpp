#include "ontologenius/core/feeder/Feeder.h"

#include <iostream>

#include "ontologenius/core/ontoGraphs/Ontology.h"

namespace ontologenius {

Feeder::Feeder(Ontology* onto, bool versioning) : versionor_(&feed_storage_)
{
  onto_ = onto;
  do_versioning_ = versioning;
}

bool Feeder::run()
{
  if(onto_ == nullptr)
    return false;

  bool has_run = false;
  std::queue<feed_t> feeds = feed_storage_.get();

  while(feeds.empty() == false)
  {
    has_run = true;
    feed_t feed = feeds.front();
    feeds.pop();

    if(feed.action_ == action_add)
      current_str_feed_ = "[add]" + feed.from_ + "|" + feed.prop_ + "|" + feed.on_;
    else if(feed.action_ == action_del)
      current_str_feed_ = "[del]" + feed.from_ + "|" + feed.prop_ + "|" + feed.on_;
    else
    {
      if(feed.action_ == action_commit)
      {
        if(!versionor_.commit(feed.from_))
          notifications_.push_back("[FAIL][commit]" + feed.from_);
      }
      else if(feed.action_ == action_checkout)
      {
        if(!versionor_.checkout(feed.from_))
          notifications_.push_back("[FAIL][checkout]" + feed.from_);
      }
      continue;
    }
    
    if(do_versioning_ && !feed.checkout_)
      versionor_.insert(feed);

    if(addFeed(feed))
      valid_relations_.emplace_back(current_str_feed_, feed.stamp);
  }

  return has_run;
}

bool Feeder::addFeed(feed_t& feed)
{
  if(feed.prop_ == "")
  {
    if(onto_->class_graph_.findBranchSafe(feed.from_) != nullptr)
      return addDelClass(feed.action_, feed.from_);
    else
      return addDelIndiv(feed.action_, feed.from_);
  }
  else if(feed.on_ != "")
  {
    if((feed.prop_ == "+") || (feed.prop_ == "rdfs:subClassOf") || (feed.prop_ == "isA"))
      return addInheritage(feed);
    else if((feed.prop_ == "<-") || (feed.prop_ == "owl:inverseOf"))
      return addInverseOf(feed);
    else if(feed.prop_[0] == '@')
      return modifyLangage(feed);
    else if((feed.prop_ == "=") || (feed.prop_ == "owl:sameAs") || (feed.prop_ == "sameAs"))
      return addSameAs(feed);
    else
      return applyProperty(feed);
  }
  else
  {
    notifications_.push_back("[FAIL][not enough arguments]" + current_str_feed_);
    return false;
  }
}

bool Feeder::addDelClass(action_t& action, std::string& name)
{
  if(action == action_add)
  {
    std::lock_guard<std::shared_timed_mutex> lock(onto_->class_graph_.mutex_);
    onto_->class_graph_.findOrCreateBranch(name);
    return true;
  }
  else
  {
    ClassBranch_t* tmp = onto_->class_graph_.findBranchSafe(name);
    onto_->class_graph_.deleteClass(tmp);
    return (tmp != nullptr);
  }
}

bool Feeder::addDelIndiv(action_t& action, std::string& name)
{
  if(action == action_add)
  {
    onto_->individual_graph_.findOrCreateBranchSafe(name);
    return true;
  }
  else
  {
    IndividualBranch_t* tmp = onto_->individual_graph_.findBranchSafe(name);
    onto_->individual_graph_.deleteIndividual(tmp);
    return (tmp != nullptr);
  }
}

bool Feeder::addInheritage(feed_t& feed)
{
  try
  {
    if(feed.action_ == action_add)
    {
      if(onto_->class_graph_.findBranchSafe(feed.from_) != nullptr)
        return onto_->class_graph_.addInheritage(feed.from_, feed.on_);
      else if(onto_->individual_graph_.findBranchSafe(feed.from_) != nullptr)
        return onto_->individual_graph_.addInheritage(feed.from_, feed.on_);
      else if(onto_->class_graph_.findBranchSafe(feed.on_) != nullptr)
        return onto_->individual_graph_.addInheritageInvert(feed.from_, feed.on_);
      else if(onto_->individual_graph_.findBranchSafe(feed.on_) != nullptr)
        return onto_->individual_graph_.addInheritageInvertUpgrade(feed.from_, feed.on_);
      if(onto_->data_property_graph_.findBranchSafe(feed.from_) != nullptr)
        return onto_->data_property_graph_.addInheritage(feed.from_, feed.on_);
      else if(onto_->data_property_graph_.findBranchSafe(feed.on_) != nullptr)
        return onto_->data_property_graph_.addInheritage(feed.from_, feed.on_);
      else if(onto_->object_property_graph_.findBranchSafe(feed.from_) != nullptr)
        return onto_->object_property_graph_.addInheritage(feed.from_, feed.on_);
      else if(onto_->object_property_graph_.findBranchSafe(feed.on_) != nullptr)
        return onto_->object_property_graph_.addInheritage(feed.from_, feed.on_);
      else
      {
        notifications_.push_back("[FAIL][no known items in the requested inheritance]" + current_str_feed_);
        return false;
      }
    }
    else if(feed.action_ == action_del)
    {
      if(onto_->class_graph_.findBranchSafe(feed.from_) != nullptr)
      {
        auto tmp = onto_->class_graph_.removeInheritage(feed.from_, feed.on_);
        explanations_.insert(explanations_.end(), tmp.begin(), tmp.end());
      }
      else if(onto_->individual_graph_.findBranchSafe(feed.from_) != nullptr)
      {
        auto tmp = onto_->individual_graph_.removeInheritage(feed.from_, feed.on_);
        explanations_.insert(explanations_.end(), tmp.begin(), tmp.end());
      }
      else if(onto_->object_property_graph_.findBranchSafe(feed.from_) != nullptr)
      {
        auto tmp = onto_->object_property_graph_.removeInheritage(feed.from_, feed.on_);
        explanations_.insert(explanations_.end(), tmp.begin(), tmp.end());
      }
      else if(onto_->data_property_graph_.findBranchSafe(feed.from_) != nullptr)
      {
        auto tmp = onto_->data_property_graph_.removeInheritage(feed.from_, feed.on_);
        explanations_.insert(explanations_.end(), tmp.begin(), tmp.end());
      }
      else
      {
        notifications_.push_back("[FAIL][unknown inherited item in the requested inheritance]" + current_str_feed_);
        return false;
      }
    }
  }
  catch(GraphException& e)
  {
    notifications_.push_back("[FAIL][" + std::string(e.what()) + "]" + current_str_feed_);
    return false;
  }
  return true;
}

bool Feeder::addInverseOf(const feed_t& feed)
{
  if(feed.action_ == action_add)
  {
    if(!onto_->object_property_graph_.addInverseOf(feed.from_, feed.on_))
    {
      notifications_.push_back("[FAIL][no known items in the request]" + current_str_feed_);
      return false;
    }
    else
      return true;
  }
  else if(feed.action_ == action_del)
  {
    if(!onto_->object_property_graph_.removeInverseOf(feed.from_, feed.on_))
    {
      notifications_.push_back("[FAIL][unknown item in the request]" + current_str_feed_);
      return false;
    }
    else
      return true;
  }
  else
    return false;
}

bool Feeder::addSameAs(const feed_t& feed)
{
  try
  {
    if(feed.action_ == action_add)
    {
      onto_->individual_graph_.addSameAs(feed.from_, feed.on_);
      return true;
    }
    else if(feed.action_ == action_del)
    {
      auto tmp = onto_->individual_graph_.removeSameAs(feed.from_, feed.on_);
      if(tmp.size() != 0)
        explanations_.insert(explanations_.end(), tmp.begin(), tmp.end());
      return true;
    }
    else
      return false;
  }
  catch(GraphException& e)
  {
    notifications_.push_back("[FAIL][" + std::string(e.what()) + "]" + current_str_feed_);
    return false;
  }
}

bool Feeder::modifyLangage(feed_t& feed)
{
  if(feed.action_ == action_add)
  {
    if(onto_->class_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->class_graph_.addLang(feed.from_, feed.prop_, feed.on_);
    else if(onto_->individual_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->individual_graph_.addLang(feed.from_, feed.prop_, feed.on_);
    else if(onto_->object_property_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->object_property_graph_.addLang(feed.from_, feed.prop_, feed.on_);
    else if(onto_->data_property_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->data_property_graph_.addLang(feed.from_, feed.prop_, feed.on_);
    else
    {
      notifications_.push_back("[FAIL][unknown element in the requested language addition]" + current_str_feed_);
      return false;
    }
  }
  else if(feed.action_ == action_del)
  {
    if(onto_->class_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->class_graph_.removeLang(feed.from_, feed.prop_, feed.on_);
    else if(onto_->individual_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->individual_graph_.removeLang(feed.from_, feed.prop_, feed.on_);
    else if(onto_->object_property_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->object_property_graph_.removeLang(feed.from_, feed.prop_, feed.on_);
    else if(onto_->data_property_graph_.findBranchSafe(feed.from_) != nullptr)
      return onto_->data_property_graph_.removeLang(feed.from_, feed.prop_, feed.on_);
    else
    {
      notifications_.push_back("[FAIL][unknown element in the requested language deletion]" + current_str_feed_);
      return false;
    }
  }
  else
    return false;
}

bool Feeder::applyProperty(feed_t& feed)
{
  size_t pose = feed.on_.find('#');
  std::string type = "";
  std::string data = "";
  bool data_property = false;

  if(pose != std::string::npos)
  {
    type = feed.on_.substr(0, pose);
    data = feed.on_.substr(pose+1);
    data_property = true;
  }

  IndividualBranch_t* indiv_branch = nullptr;
  ClassBranch_t* class_branch = nullptr;

  try {

    if(feed.action_ == action_add)
    {
      if((indiv_branch = onto_->individual_graph_.findBranchSafe(feed.from_)) != nullptr)
      {
        if(data_property == true)
          onto_->individual_graph_.addRelation(indiv_branch, feed.prop_, type, data);
        else
          onto_->individual_graph_.addRelation(indiv_branch, feed.prop_, feed.on_);
      }
      else if((class_branch = onto_->class_graph_.findBranchSafe(feed.from_)) != nullptr)
      {
        if(data_property == true)
          onto_->class_graph_.addRelation(class_branch, feed.prop_, type, data);
        else
          onto_->class_graph_.addRelation(class_branch, feed.prop_, feed.on_);
      }
      else if((class_branch = onto_->class_graph_.findBranchSafe(feed.on_)) != nullptr)
        onto_->class_graph_.addRelationInvert(feed.from_, feed.prop_, class_branch);
      else if((indiv_branch = onto_->individual_graph_.findBranchSafe(feed.on_)) != nullptr)
        onto_->individual_graph_.addRelationInvert(feed.from_, feed.prop_, indiv_branch);
      else
      {
        notifications_.push_back("[FAIL][unknown concept to apply property]" + current_str_feed_);
        return false;
      }
    }
    else if(feed.action_ == action_del)
    {
      if(onto_->class_graph_.findBranchSafe(feed.from_) != nullptr)
      {
        if(data_property == true)
          onto_->class_graph_.removeRelation(feed.from_, feed.prop_, type, data);
        else
          onto_->class_graph_.removeRelation(feed.from_, feed.prop_, feed.on_);
      }
      else if(onto_->individual_graph_.findBranchSafe(feed.from_) != nullptr)
      {
        if(data_property == true)
        {
          auto tmp = onto_->individual_graph_.removeRelation(feed.from_, feed.prop_, type, data);
          explanations_.insert(explanations_.end(), tmp.begin(), tmp.end());
        }
        else
        {
          auto tmp = onto_->individual_graph_.removeRelation(feed.from_, feed.prop_, feed.on_);
          explanations_.insert(explanations_.end(), tmp.begin(), tmp.end());
        }
      }
      else
      {
        notifications_.push_back("[FAIL][unknown concept to remove property]" + current_str_feed_);
        return false;
      }
    }
    else
    {
      notifications_.push_back("[FAIL][unknown action]" + current_str_feed_);
      return false;
    }
  }
  catch(GraphException& e)
  {
    notifications_.push_back("[FAIL][" + std::string(e.what()) + "]" + current_str_feed_);
    return false;
  }

  return true;
}

} // namespace ontologenius
