#ifndef ARGUERS_H
#define ARGUERS_H

#include "ontoloGenius/ontoGraphs/Ontology.h"
#include "ontoloGenius/arguer/plugins/ArguerInterface.h"

#include <pluginlib/class_loader.h>

#include <map>
#include <string>

class Arguers
{
public:
  Arguers(Ontology* onto) : loader_("ontologenius", "ArguerInterface") {ontology_ = onto; }
  ~Arguers();

  void load();
  void list();

  int activate(std::string plugin);
  int deactivate(std::string plugin);

  void runPreArguers();
  void runPostArguers();
private:
  Ontology* ontology_;
  std::map<std::string, ArguerInterface*> arguers_;
  std::map<std::string, ArguerInterface*> active_arguers_;

  pluginlib::ClassLoader<ArguerInterface> loader_;
};

#endif
