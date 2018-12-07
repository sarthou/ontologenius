#include "ontoloGenius/core/feeder/Feeder.h"
#include "ontoloGenius/core/ontoGraphs/Ontology.h"

#include <iostream>

void Feeder::run()
{
  std::queue<feed_t> feeds = feed_storage_.get();
  while(feeds.empty() == false)
  {
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
    }
    std::cout << feed.from_ << " : " << feed.prop_ << " : " << feed.on_ << std::endl;
  }
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
    onto_->individual_graph_.create(name);
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
