#include "ontologenius/API/ontologenius/OntologyManipulator.h"
#include "ontologenius/graphical/Display.h"

OntologyManipulator::OntologyManipulator(ros::NodeHandle* n, const std::string& name) : name_(name),
                                                                                        individuals(&n_, name),
                                                                                        objectProperties(&n_, name),
                                                                                        dataProperties(&n_, name),
                                                                                        classes(&n_, name),
                                                                                        actions(&n_, name),
                                                                                        reasoners(&n_, name),
                                                                                        feeder(&n_, name),
                                                                                        sparql(&n_, name)
{
  (void)n;
  ontologenius::Display::warning("OntologyManipulator(ros::NodeHandle* n, const std::string& name) is deprecated. Use OntologyManipulator(const std::string& name) instead.");
  name_ = name;
  std::string servive_name = (name == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name;
  ros::service::waitForService(servive_name);
}

OntologyManipulator::OntologyManipulator(const std::string& name) : name_(name),
                                                                    individuals(&n_, name),
                                                                    objectProperties(&n_, name),
                                                                    dataProperties(&n_, name),
                                                                    classes(&n_, name),
                                                                    actions(&n_, name),
                                                                    reasoners(&n_, name),
                                                                    feeder(&n_, name),
                                                                    sparql(&n_, name)
{
  std::string servive_name = (name == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name;
  ros::service::waitForService(servive_name);
}

OntologyManipulator::OntologyManipulator(const OntologyManipulator& other): name_(other.name_),
                                                                      individuals(&n_, other.name_),
                                                                      objectProperties(&n_, other.name_),
                                                                      dataProperties(&n_, other.name_),
                                                                      classes(&n_, other.name_),
                                                                      actions(&n_, other.name_),
                                                                      reasoners(&n_, other.name_),
                                                                      feeder(&n_, other.name_),
                                                                      sparql(&n_, other.name_)
{
  std::string servive_name = (name_ == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name_;
  ros::service::waitForService(servive_name);
}

OntologyManipulator::OntologyManipulator(OntologyManipulator&& other): name_(other.name_),
                                                                      individuals(&n_, other.name_),
                                                                      objectProperties(&n_, other.name_),
                                                                      dataProperties(&n_, other.name_),
                                                                      classes(&n_, other.name_),
                                                                      actions(&n_, other.name_),
                                                                      reasoners(&n_, other.name_),
                                                                      feeder(&n_, other.name_),
                                                                      sparql(&n_, other.name_)
{
  std::string servive_name = (name_ == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name_;
  ros::service::waitForService(servive_name);
}
