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

    if(feed.prop_ == "")
    {
      if(onto_->class_graph_.findBranch(feed.from_) != nullptr)
        addDelClass(feed.action_, feed.from_);
      else
        addDelIndiv(feed.action_, feed.from_);
    }
    std::cout << feed.from_ << " : " << feed.prop_ << " : " << feed.on_ << std::endl;
  }
}

void Feeder::addDelClass(action_t action, std::string name)
{
  if(action == action_add)
    onto_->class_graph_.create(name);
  else
  {
    ClassBranch_t* tmp = onto_->class_graph_.findBranch(name);
  }
}

void Feeder::addDelIndiv(action_t action, std::string name)
{
  if(action == action_add)
    onto_->individual_graph_.create(name);
  else
  {
    IndividualBranch_t* tmp = onto_->individual_graph_.findBranch(name);
    onto_->individual_graph_.deleteIndividual(tmp);
  }
}
