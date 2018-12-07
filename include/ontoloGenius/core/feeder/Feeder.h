#ifndef FEEDER_H
#define FEEDER_H

#include "ontoloGenius/core/feeder/FeedStorage.h"

class Ontology;

class Feeder
{
public:
  Feeder(Ontology* onto) {onto_ = onto; }

  void store(std::string feed) { feed_storage_.add(feed); }
  void run();
  void link(Ontology* onto) {onto_ = onto; }

private:
  FeedStorage feed_storage_;
  Ontology* onto_;

  void addDelClass(action_t& action, std::string& name);
  void addDelIndiv(action_t& action, std::string& name);
  void modifyDataProperty(feed_t& feed);
};

#endif
