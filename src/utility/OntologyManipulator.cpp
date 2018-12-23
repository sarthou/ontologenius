#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator::OntologyManipulator(ros::NodeHandle* n, const std::string& name) : individuals(n, name),
                                                                                        objectProperties(n, name),
                                                                                        dataProperties(n, name),
                                                                                        classes(n, name),
                                                                                        actions(n, name),
                                                                                        reasoners(n, name),
                                                                                        feeder(n, name)
{
  n_ = n;
  std::string servive_name = (name == "") ? "ontologenius/reasoner" : "ontologenius/reasoner/" + name;
  ros::service::waitForService(servive_name);
}
