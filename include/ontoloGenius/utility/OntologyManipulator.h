#ifndef ONTOLOGYMANIPULATOR_H
#define ONTOLOGYMANIPULATOR_H

#include "ontologenius/OntologeniusService.h"

#include "ros/ros.h"

#include <vector>
#include <string>

class OntologyManipulator
{
public:
  OntologyManipulator(ros::NodeHandle* n);
  ~OntologyManipulator() {}

  std::vector<std::string> string2vector(const std::string& value);

  std::vector<std::string> getOn(const std::string& name, const std::string& property);
  std::vector<std::string> getFrom(const std::string& property, const std::string& name, const std::string& selector = "");
  std::vector<std::string> getWith(const std::string& indiv_1, const std::string& indiv_2, const std::string& selector = "");
  std::vector<std::string> getUp(std::string& name, int depth = -1, const std::string& selector = "");
  bool isA(std::string& name, const std::string& base_class);
  std::vector<std::string> getDown(std::string& name);
  std::vector<std::string> getRelatedFrom(const std::string& name);
  std::vector<std::string> getType(const std::string& name);
  std::string getName(const std::string& name);

  size_t nb() {return cpt;}
  void reset() {cpt = 0;}

  bool close();

private:
  ros::NodeHandle* n_;
  size_t cpt;

  ros::ServiceClient class_client;
  ros::ServiceClient object_property_client;
  ros::ServiceClient data_property_client;
  ros::ServiceClient individual_client;
  ros::ServiceClient arguer_client;


  inline std::vector<std::string> callIndividual(ontologenius::OntologeniusService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(individual_client.call(srv))
      return srv.response.values;
    else
      return res;
  }

  inline std::vector<std::string> callObjectProperty(ontologenius::OntologeniusService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(object_property_client.call(srv))
      return srv.response.values;
    else
      return res;
  }

  inline std::vector<std::string> callDataProperty(ontologenius::OntologeniusService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(data_property_client.call(srv))
      return srv.response.values;
    else
      return res;
  }

  inline std::vector<std::string> callClass(ontologenius::OntologeniusService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(class_client.call(srv))
      return srv.response.values;
    else
      return res;
  }

  inline std::vector<std::string> callArguer(ontologenius::OntologeniusService& srv)
  {
    std::vector<std::string> res;
    cpt++;

    if(arguer_client.call(srv))
      return srv.response.values;
    else
      return res;
  }
};

#endif
