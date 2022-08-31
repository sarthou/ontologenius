#ifndef ONTOLOGENIUS_REASONERINTERFACE_H
#define ONTOLOGENIUS_REASONERINTERFACE_H

#include <string>

#include "ontologenius/core/ontoGraphs/Ontology.h"

namespace ontologenius {

enum ReasonerNotificationStatus_e
{
  notification_debug,
  notification_info,
  notification_warning,
  notification_error
};

enum QueryType_e
{
  query_relation,
  query_inheritance,
  query_label,
  query_other
};

enum QueryOrigin_e
{
  query_origin_individual,
  query_origin_class,
  query_origin_object_property,
  query_origin_data_property,
};

struct QueryInfo_t
{
  QueryType_e query_type;
  QueryOrigin_e query_origin;
  std::string subject;
  std::string predicate;
  std::string object;
};

class ReasonerInterface
{
public:
  virtual ~ReasonerInterface() = default;

  void initialize(const std::string& agent_name, Ontology* onto)
  {
    agent_name_ = agent_name;
    ontology_ = onto;
  }
  virtual void initialize() {} // This function is called once the ontology is closed 
  virtual void setParameter(const std::string& name, const std::string& value)
  {
    (void)name;
    (void)value;
  }

  virtual void preReason(const QueryInfo_t& query_info)
  {
    (void)query_info;
  }
  virtual void postReason() {}
  virtual void periodicReason() {}

  virtual bool implementPostReasoning() { return false; }
  virtual bool implementPreReasoning() { return false; }
  virtual bool implementPeriodicReasoning() { return false; }

  virtual std::string getName() = 0;
  virtual std::string getDesciption() = 0;

  virtual bool defaultAvtive() {return false;}

  static size_t getNbUpdates() {return nb_update_; }
  static void resetNbUpdates() {nb_update_ = 0; }

  std::vector<std::pair<ReasonerNotificationStatus_e, std::string>> getNotifications()
  {
    auto tmp = std::move(notifications_);
    notifications_.clear();
    return tmp;
  }

  std::vector<std::pair<std::string, std::string>> getExplanations()
  {
    auto tmp = std::move(explanations_);
    explanations_.clear();
    return tmp;
  }
protected:
  ReasonerInterface() : ontology_(nullptr) {}

  std::string agent_name_;
  Ontology* ontology_;

  std::vector<std::pair<ReasonerNotificationStatus_e, std::string>> notifications_;
  std::vector<std::pair<std::string, std::string>> explanations_;

  static size_t nb_update_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERINTERFACE_H
