#ifndef ONTOLOGENIUS_SPARQLCLIENT_H
#define ONTOLOGENIUS_SPARQLCLIENT_H

#include <ros/ros.h>

#include "ontologenius/OntologeniusSparqlService.h"
#include "ontologenius/OntologeniusSparqlResponse.h"

class SparqlClient
{
public:
  SparqlClient(ros::NodeHandle* n, const std::string& name) : client(n->serviceClient<ontologenius::OntologeniusSparqlService>((name == "") ? "ontologenius/sparql" : "ontologenius/sparql/" + name, true))
  {
    name_ = (name == "") ? "sparql" : "sparql/" + name;
    n_ = n;
  }

  std::vector<ontologenius::OntologeniusSparqlResponse> call(const std::string& query);

private:
  ros::ServiceClient client;
  std::string name_;
  ros::NodeHandle* n_;
};

#endif // ONTOLOGENIUS_SPARQLCLIENT_H
