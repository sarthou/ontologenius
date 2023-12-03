#include "ontologenius/API/ontologenius/OntologyManipulator.h"
#include "ontologenius/graphical/Display.h"

namespace onto {

OntologyManipulator::OntologyManipulator(ros::NodeHandle* n, const std::string& name) : name_(name),
                                                                                        individuals(name),
                                                                                        objectProperties(name),
                                                                                        dataProperties(name),
                                                                                        classes(name),
                                                                                        actions(name),
                                                                                        reasoners(name),
                                                                                        feeder(name),
                                                                                        sparql(name)
{
  (void)n;
  ontologenius::Display::warning("OntologyManipulator(ros::NodeHandle* n, const std::string& name) is deprecated. Use OntologyManipulator(const std::string& name) instead.");
  name_ = name;
  std::string service_name = (name == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name;
  ros::service::waitForService(service_name);
}

OntologyManipulator::OntologyManipulator(const std::string& name) : name_(name),
                                                                    individuals(name),
                                                                    objectProperties(name),
                                                                    dataProperties(name),
                                                                    classes(name),
                                                                    actions(name),
                                                                    reasoners(name),
                                                                    feeder(name),
                                                                    sparql(name)
{
  std::string service_name = (name == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name;
  ros::service::waitForService(service_name);
}

OntologyManipulator::OntologyManipulator(const OntologyManipulator& other): name_(other.name_),
                                                                      individuals(other.name_),
                                                                      objectProperties(other.name_),
                                                                      dataProperties(other.name_),
                                                                      classes(other.name_),
                                                                      actions(other.name_),
                                                                      reasoners(other.name_),
                                                                      feeder(other.name_),
                                                                      sparql(other.name_)
{
  std::string service_name = (name_ == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name_;
  ros::service::waitForService(service_name);
}

OntologyManipulator::OntologyManipulator(OntologyManipulator&& other): name_(other.name_),
                                                                      individuals(other.name_),
                                                                      objectProperties(other.name_),
                                                                      dataProperties(other.name_),
                                                                      classes(other.name_),
                                                                      actions(other.name_),
                                                                      reasoners(other.name_),
                                                                      feeder(other.name_),
                                                                      sparql(other.name_)
{
  std::string service_name = (name_ == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name_;
  ros::service::waitForService(service_name);
}

} // namespace onto