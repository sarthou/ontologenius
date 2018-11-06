#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator::OntologyManipulator(ros::NodeHandle* n) :  individuals(n),
                                                                objectProperties(n),
                                                                dataProperties(n),
                                                                classes(n),
                                                                actions(n)
{
  n_ = n;
  ros::service::waitForService("ontologenius/arguer");
}
