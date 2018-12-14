#ifndef FEEDER_H
#define FEEDER_H

#include "ontoloGenius/core/feeder/FeedStorage.h"

class Ontology;

class Feeder
{
public:
  Feeder(Ontology* onto) {onto_ = onto; }

  void store(std::string feed) { feed_storage_.add(feed); }
  bool run();
  void link(Ontology* onto) {onto_ = onto; }

  std::vector<std::string> getNotifications()
  {
    std::vector<std::string> tmp = notifications_;
    notifications_.clear();
    return tmp;
  }

private:
  FeedStorage feed_storage_;
  Ontology* onto_;

  std::vector<std::string> notifications_;
  std::string current_str_feed_;

  void addDelClass(action_t& action, std::string& name);
  void addDelIndiv(action_t& action, std::string& name);
  void modifyDataProperty(feed_t& feed);
  void modifyDataPropertyInvert(feed_t& feed);
  void modifyObjectProperty(feed_t& feed);
  void modifyObjectPropertyInvert(feed_t& feed);
  void classIndividualIsA(feed_t& feed);
  void classIndividualLangage(feed_t& feed);
  void applyProperty(feed_t& feed);
};

#endif
