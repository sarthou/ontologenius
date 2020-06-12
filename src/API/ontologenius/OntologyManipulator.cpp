#include "ontologenius/API/ontologenius/OntologyManipulator.h"

OntologyManipulator::OntologyManipulator(ros::NodeHandle* n, const std::string& name) : individuals(n, name),
                                                                                        objectProperties(n, name),
                                                                                        dataProperties(n, name),
                                                                                        classes(n, name),
                                                                                        actions(n, name),
                                                                                        reasoners(n, name),
                                                                                        feeder(n, name),
                                                                                        sparql(n, name)
{
  n_ = n;
  name_ = name;
  std::string servive_name = (name == "") ? "ontologenius/reasoner" : "ontologenius/sparql/" + name;
  ros::service::waitForService(servive_name);
}

OntologyManipulator::OntologyManipulator(OntologyManipulator& other): individuals(other.n_, other.name_),
                                                                      objectProperties(other.n_, other.name_),
                                                                      dataProperties(other.n_, other.name_),
                                                                      classes(other.n_, other.name_),
                                                                      actions(other.n_, other.name_),
                                                                      reasoners(other.n_, other.name_),
                                                                      feeder(other.n_, other.name_),
                                                                      sparql(other.n_, other.name_)
{
  n_ = other.n_;
  name_ = other.name_;
  std::string servive_name = (name_ == "") ? "ontologenius/reasoner" : "ontologenius/sparql/" + name_;
  ros::service::waitForService(servive_name);
}

OntologyManipulator::OntologyManipulator(OntologyManipulator&& other): individuals(other.n_, other.name_),
                                                                      objectProperties(other.n_, other.name_),
                                                                      dataProperties(other.n_, other.name_),
                                                                      classes(other.n_, other.name_),
                                                                      actions(other.n_, other.name_),
                                                                      reasoners(other.n_, other.name_),
                                                                      feeder(other.n_, other.name_),
                                                                      sparql(other.n_, other.name_)
{
  n_ = other.n_;
  name_ = other.name_;
  std::string servive_name = (name_ == "") ? "ontologenius/reasoner" : "ontologenius/sparql/" + name_;
  ros::service::waitForService(servive_name);
}
