#ifndef ONTOLOGENIUS_REASONERS_H
#define ONTOLOGENIUS_REASONERS_H

#include <map>
#include <string>

#include <pluginlib/class_loader.h>

#include "ontoloGenius/core/ontoGraphs/Ontology.h"
#include "ontoloGenius/core/reasoner/ConfigReader.h"
#include "ontoloGenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class Reasoners
{
public:
  Reasoners(Ontology* onto = nullptr);
  ~Reasoners();

  void link(Ontology* onto);

  void configure(const std::string& config_path);
  void load();
  std::string list();
  std::vector<std::string> listVector();
  std::vector<std::string> activeListVector();

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
  ConfigReader config_;
  std::map<std::string, ReasonerInterface*> reasoners_;
  std::map<std::string, ReasonerInterface*> active_reasoners_;
  std::vector<std::string> notifications_;

  pluginlib::ClassLoader<ReasonerInterface> loader_;

  void applyConfig();

  void computeIndividualsUpdates();
  void computeIndividualsUpdatesPeriodic();
  void resetIndividualsUpdates();
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERS_H
