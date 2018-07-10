#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator::OntologyManipulator(ros::NodeHandle* n) :  individuals(n)
{
  n_ = n;
}

bool OntologyManipulator::close()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/actions");
  ontologenius::OntologeniusService srv;
  srv.request.action = "close";

  if(!client.call(srv))
    return false;
  else
    return true;
}
