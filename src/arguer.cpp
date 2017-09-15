#include "ontoloGenius/tree.h"
#include "ontoloGenius/tree_drawer.h"
#include "ontoloGenius/ontology_reader.h"

#include "ontologenius/standard_service.h"
#include "ontoloGenius/utility/error_code.h"

#include <iostream>
#include "ros/ros.h"

#include "ontoloGenius/computer.h"

using namespace std;

tree onto;

bool reference_handle(ontologenius::standard_service::Request  &req,
                      ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(req.action == "add")
  {
    Ontology_reader reader(&onto);
    res.code = reader.read(req.param, type_class);
    res.code = reader.read(req.param, type_individual);
  }
  else if(req.action == "close")
  {
    onto.close();
  }
  else if(req.action == "getDown")
  {
    set<string> entities = onto.getDown(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else if(req.action == "getUp")
  {
    set<string> entities = onto.getUp(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else if(req.action == "reset")
  {
    onto = tree();
  }
  else if(req.action == "test")
  {
    computer comp;
    if(comp.compute(req.param, onto))
      res.value = "true";
    else
      res.value = "false";
    //comp.compute("red_cube|young_animal=color_animal|age_object", onto);
  }
  else
    res.code = UNKNOW_ACTION;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontoloGenius");

  ros::NodeHandle n;

  ros::service::waitForService("ontoloGenius/REST", -1);

  for(unsigned int i = 1; i < argc; i++)
  {
    Ontology_reader reader(&onto);
    reader.readFile(string(argv[i]), type_class);
    reader.readFile(argv[i], type_individual);
  }

  // Start up ROS service with callbacks
  ros::ServiceServer service = n.advertiseService("ontoloGenius/actions", reference_handle);
  ROS_DEBUG("ontoloGenius ready");

  ros::spin();

  ROS_DEBUG("KILL ontoloGenius");

  return 0;
}
