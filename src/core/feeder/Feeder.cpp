#include "ontoloGenius/core/feeder/Feeder.h"
#include "ontoloGenius/core/ontoGraphs/Ontology.h"

#include <iostream>

bool Feeder::run()
{
  bool has_run = false;
  std::queue<feed_t> feeds = feed_storage_.get();
  while(feeds.empty() == false)
  {
    has_run = true;
    feed_t feed = feeds.front();
    feeds.pop();
    bool done = false;

    if(feed.prop_ == "")
    {
      if(onto_->class_graph_.findBranch(feed.from_) != nullptr)
        addDelClass(feed.action_, feed.from_);
      else
        addDelIndiv(feed.action_, feed.from_);
    }
    else if(feed.on_ != "")
    {
      if(onto_->data_property_graph_.findBranch(feed.from_) != nullptr)
        done = modifyDataProperty(feed);
      else if(onto_->data_property_graph_.findBranch(feed.on_) != nullptr)
        done = modifyDataPropertyInvert(feed);
      else if(onto_->object_property_graph_.findBranch(feed.from_) != nullptr)
        done = modifyObjectProperty(feed);
      else if(onto_->object_property_graph_.findBranch(feed.on_) != nullptr)
        done = modifyObjectPropertyInvert(feed);
      else if((feed.prop_ == "+") || (feed.prop_ == "isA"))
        done = classIndividualIsA(feed);
      else if(feed.prop_[0] == '@')
        done = classIndividualLangage(feed);
      else
        done = applyProperty(feed);
    }
    std::cout << feed.from_ << " : " << feed.prop_ << " : " << feed.on_ << std::endl;
  }

  return has_run;
}

void Feeder::addDelClass(action_t& action, std::string& name)
{
  if(action == action_add)
    onto_->class_graph_.create(name);
  else
  {
    ClassBranch_t* tmp = onto_->class_graph_.findBranch(name);
    onto_->class_graph_.deleteClass(tmp);
  }
}

void Feeder::addDelIndiv(action_t& action, std::string& name)
{
  if(action == action_add)
    onto_->individual_graph_.createIndividual(name);
  else
  {
    IndividualBranch_t* tmp = onto_->individual_graph_.findBranch(name);
    onto_->individual_graph_.deleteIndividual(tmp);
  }
}

bool Feeder::modifyDataProperty(feed_t& feed)
{
  DataPropertyBranch_t* tmp = onto_->data_property_graph_.findBranch(feed.from_);
  if(feed.action_ == action_add)
    return onto_->data_property_graph_.add(tmp, feed.prop_, feed.on_);
  else
    return onto_->data_property_graph_.remove(tmp, feed.prop_, feed.on_);
}

bool Feeder::modifyDataPropertyInvert(feed_t& feed)
{
  DataPropertyBranch_t* tmp = onto_->data_property_graph_.findBranch(feed.on_);
  if(feed.action_ == action_add)
    return onto_->data_property_graph_.addInvert(tmp, feed.prop_, feed.from_);
  else
    return false;
}

bool Feeder::modifyObjectProperty(feed_t& feed)
{
  ObjectPropertyBranch_t* tmp = onto_->object_property_graph_.findBranch(feed.from_);
  if(feed.action_ == action_add)
    return onto_->object_property_graph_.add(tmp, feed.prop_, feed.on_);
  else
    return onto_->object_property_graph_.remove(tmp, feed.prop_, feed.on_);
}

bool Feeder::modifyObjectPropertyInvert(feed_t& feed)
{
  ObjectPropertyBranch_t* tmp = onto_->object_property_graph_.findBranch(feed.on_);
  if(feed.action_ == action_add)
    return onto_->object_property_graph_.addInvert(tmp, feed.prop_, feed.from_);
  else
    return false;
}

bool Feeder::classIndividualIsA(feed_t& feed)
{
  if(onto_->class_graph_.findBranch(feed.from_) != nullptr)
    onto_->class_graph_.addInheritage(feed.from_, feed.on_);
  else if(onto_->individual_graph_.findBranch(feed.from_) != nullptr)
    onto_->individual_graph_.addInheritage(feed.from_, feed.on_);
  else if(onto_->class_graph_.findBranch(feed.on_) != nullptr)
    onto_->individual_graph_.addInheritageInvert(feed.from_, feed.on_);
  else if(onto_->individual_graph_.findBranch(feed.on_) != nullptr)
    onto_->individual_graph_.addInheritageInvertUpgrade(feed.from_, feed.on_);

  else
    return false;
  return true;
}

bool Feeder::classIndividualLangage(feed_t& feed)
{
  if(onto_->class_graph_.findBranch(feed.from_) != nullptr)
    onto_->class_graph_.addLang(feed.from_, feed.prop_, feed.on_);
  else if(onto_->individual_graph_.findBranch(feed.from_) != nullptr)
    onto_->individual_graph_.addLang(feed.from_, feed.prop_, feed.on_);
  else
    return false;
  return true;
}

bool Feeder::applyProperty(feed_t& feed)
{
  size_t pose = feed.on_.find(":");
  std::string type = "";
  std::string data = "";
  bool data_property = false;

  if(pose != std::string::npos)
  {
    type = feed.on_.substr(0, pose);
    data = feed.on_.substr(pose+1);
    data_property = true;
  }

  if(feed.action_ == action_add)
  {
    if(onto_->class_graph_.findBranch(feed.from_) != nullptr)
    {
      if(data_property == true)
        return onto_->class_graph_.addProperty(feed.from_, feed.prop_, type, data);
      else
        return onto_->class_graph_.addProperty(feed.from_, feed.prop_, feed.on_);
    }
    else if(onto_->individual_graph_.findBranch(feed.from_) != nullptr)
    {
      if(data_property == true)
        return onto_->individual_graph_.addProperty(feed.from_, feed.prop_, type, data);
      else
        return onto_->individual_graph_.addProperty(feed.from_, feed.prop_, feed.on_);
    }
    else if(onto_->class_graph_.findBranch(feed.on_) != nullptr)
      return onto_->class_graph_.addPropertyInvert(feed.from_, feed.prop_, feed.on_);
    else if(onto_->individual_graph_.findBranch(feed.on_) != nullptr)
      return onto_->individual_graph_.addPropertyInvert(feed.from_, feed.prop_, feed.on_);
  }
  return false;
}
