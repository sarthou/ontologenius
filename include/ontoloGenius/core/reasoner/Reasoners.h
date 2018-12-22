#ifndef REASONERS_H
#define REASONERS_H

#include "ontoloGenius/core/ontoGraphs/Ontology.h"
#include "ontoloGenius/core/reasoner/plugins/ReasonerInterface.h"

#include <pluginlib/class_loader.h>

#include <map>
#include <string>

class Reasoners
{
public:
  Reasoners(Ontology* onto);
  ~Reasoners();

  void link(Ontology* onto);

  void load();
  std::string list();
  std::vector<std::string> listVector();

  int activate(std::string plugin);
  int deactivate(std::string plugin);
  std::string getDescription(std::string& plugin);

  void runPreReasoners();
  void runPostReasoners();
  void runPeriodicReasoners();

  std::vector<std::string> getNotifications()
  {
    std::vector<std::string> tmp = notifications_;
    notifications_.clear();
    return tmp;
  }

private:
  Ontology* ontology_;
  std::map<std::string, ReasonerInterface*> reasoners_;
  std::map<std::string, ReasonerInterface*> active_reasoners_;
  std::vector<std::string> notifications_;

  pluginlib::ClassLoader<ReasonerInterface> loader_;

  void computeIndividualsUpdates();
  void resetIndividualsUpdates();
};

#endif
