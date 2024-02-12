#ifndef ONTOLOGENIUS_REASONERS_H
#define ONTOLOGENIUS_REASONERS_H

#include <map>
#include <string>

#include <pluginlib/class_loader.hpp>

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/reasoner/ConfigReader.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class Reasoners
{
public:
  explicit Reasoners(const std::string& agent_name, Ontology* onto = nullptr);
  ~Reasoners();

  void link(Ontology* onto);

  void configure(const std::string& config_path);
  void load();
  void initialize();
  std::string list();
  std::vector<std::string> listVector();
  std::vector<std::string> activeListVector();

  int activate(const std::string& plugin);
  int deactivate(const std::string& plugin);
  std::string getDescription(std::string& plugin);

  void runPreReasoners(QueryOrigin_e origin, const std::string& action, const std::string& param);
  void runPostReasoners();
  void runPeriodicReasoners();

  std::vector<std::pair<ReasonerNotificationStatus_e, std::string>> getNotifications()
  {
    auto tmp = std::move(notifications_);
    notifications_.clear();
    return tmp;
  }

  std::vector<std::pair<std::string, std::string>> getExplanations()
  {
    explanations_mutex_.lock();
    auto tmp = std::move(explanations_);
    explanations_.clear();
    explanations_mutex_.unlock();
    return tmp;
  }

private:
  std::string agent_name_;
  Ontology* ontology_;
  ConfigReader config_;
  std::map<std::string, ReasonerInterface*> reasoners_;
  std::map<std::string, ReasonerInterface*> active_reasoners_;
  std::vector<std::pair<ReasonerNotificationStatus_e, std::string>> notifications_;
  // Here the explanations are about relations added through FOL
  std::vector<std::pair<std::string, std::string>> explanations_;
  std::mutex explanations_mutex_;

  pluginlib::ClassLoader<ReasonerInterface> loader_;

  void applyConfig();

  void computeUpdates();
  template<typename B> void computeGraphUpdates(Graph<B>* graph);
  void computeIndividualsUpdates();
  void computeClassesUpdates();
  void computeIndividualsUpdatesPeriodic();
  void resetIndividualsUpdates();

  QueryInfo_t extractQueryInfo(QueryOrigin_e origin, const std::string& action, const std::string& param);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERS_H
