#include "ontologenius/API/ontologenius/OntologyManipulatorIndex.h"
#include "ontologenius/graphical/Display.h"

namespace onto {

OntologyManipulatorIndex::OntologyManipulatorIndex(const std::string& name) : name_(name),
                                                                    individuals(&n_, name),
                                                                    objectProperties(&n_, name),
                                                                    dataProperties(&n_, name),
                                                                    classes(&n_, name),
                                                                    actions(&n_, name),
                                                                    reasoners(&n_, name),
                                                                    feeder(&n_, name),
                                                                    sparql(&n_, name),
                                                                    conversion(&n_, name)
{
  std::string service_name = (name == "") ? "ontologenius/conversion" : "ontologenius/conversion/" + name;
  ros::service::waitForService(service_name);
}

OntologyManipulatorIndex::OntologyManipulatorIndex(const OntologyManipulatorIndex& other): name_(other.name_),
                                                                      individuals(&n_, other.name_),
                                                                      objectProperties(&n_, other.name_),
                                                                      dataProperties(&n_, other.name_),
                                                                      classes(&n_, other.name_),
                                                                      actions(&n_, other.name_),
                                                                      reasoners(&n_, other.name_),
                                                                      feeder(&n_, other.name_),
                                                                      sparql(&n_, other.name_),
                                                                      conversion(&n_, other.name_)
{
  std::string service_name = (name_ == "") ? "ontologenius/conversion" : "ontologenius/conversion/" + name_;
  ros::service::waitForService(service_name);
}

OntologyManipulatorIndex::OntologyManipulatorIndex(OntologyManipulatorIndex&& other): name_(other.name_),
                                                                      individuals(&n_, other.name_),
                                                                      objectProperties(&n_, other.name_),
                                                                      dataProperties(&n_, other.name_),
                                                                      classes(&n_, other.name_),
                                                                      actions(&n_, other.name_),
                                                                      reasoners(&n_, other.name_),
                                                                      feeder(&n_, other.name_),
                                                                      sparql(&n_, other.name_),
                                                                      conversion(&n_, other.name_)
{
  std::string service_name = (name_ == "") ? "ontologenius/conversion" : "ontologenius/conversion/" + name_;
  ros::service::waitForService(service_name);
}

} // namespace onto
