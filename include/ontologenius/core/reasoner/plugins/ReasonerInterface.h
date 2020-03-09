#ifndef ONTOLOGENIUS_REASONERINTERFACE_H
#define ONTOLOGENIUS_REASONERINTERFACE_H

#include <string>

#include "ontologenius/core/ontoGraphs/Ontology.h"

namespace ontologenius {

class ReasonerInterface
{
public:
  virtual ~ReasonerInterface() {}

  virtual void initialize(Ontology* onto) {ontology_ = onto; }
  virtual void setParameter(const std::string& name, const std::string& value)
  {
    (void)name;
    (void)value;
  }

  virtual void preReason() {}
  virtual void postReason() {}
  virtual void periodicReason() {}

  virtual std::string getName() = 0;
  virtual std::string getDesciption() = 0;

  virtual bool defaultAvtive() {return false;}

  static size_t getNbUpdates() {return nb_update_; }
  static void resetNbUpdates() {nb_update_ = 0; }

  std::vector<std::string> getNotifications()
  {
    std::vector<std::string> tmp = notifications_;
    notifications_.clear();
    return tmp;
  }
protected:
  ReasonerInterface() { }

  Ontology* ontology_;

  std::vector<std::string> notifications_;

  static size_t nb_update_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERINTERFACE_H
