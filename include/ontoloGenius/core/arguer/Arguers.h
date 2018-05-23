#ifndef ARGUERS_H
#define ARGUERS_H

#include "ontoloGenius/core/ontoGraphs/Ontology.h"
#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

#include <pluginlib/class_loader.h>

#include <map>
#include <string>

class Arguers
{
public:
  Arguers(Ontology* onto) : loader_("ontologenius", "ArguerInterface") {ontology_ = onto; }
  ~Arguers();

  void load();
  std::string list();
  std::vector<std::string> listVector();

  int activate(std::string plugin);
  int deactivate(std::string plugin);
  std::string getDescription(std::string plugin);

  void runPreArguers();
  void runPostArguers();

private:
  Ontology* ontology_;
  std::map<std::string, ArguerInterface*> arguers_;
  std::map<std::string, ArguerInterface*> active_arguers_;

  pluginlib::ClassLoader<ArguerInterface> loader_;

  void computeIndividualsUpdates();
};

#endif
