#include "ontoloGenius/treeObject.h"
#include "ontoloGenius/treeProperty.h"
#include "ontoloGenius/tree_drawer.h"
#include "ontoloGenius/ontology_reader.h"

#include "ontologenius/standard_service.h"
#include "ontoloGenius/utility/error_code.h"

#include <iostream>
#include "ros/ros.h"

#include "ontoloGenius/computer.h"

using namespace std;

treeObject onto;
treeProperty propOnto(&onto);

bool reference_handle(ontologenius::standard_service::Request  &req,
                      ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(req.action == "add")
  {
    Ontology_reader reader(&onto, &propOnto);
    res.code = reader.read(req.param);
  }
  else if(req.action == "close")
  {
    onto.close();
  }
  else if(req.action == "reset")
  {
    onto = treeObject();
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

bool class_handle(ontologenius::standard_service::Request  &req,
                  ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

if(req.action == "getDown")
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
  else if(req.action == "getDisjoint")
  {
    set<string> entities = onto.getDisjoint(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else
    res.code = UNKNOW_ACTION;

  return true;
}

bool property_handle(ontologenius::standard_service::Request  &req,
                    ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

if(req.action == "getDown")
  {
    set<string> entities = propOnto.getDown(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else if(req.action == "getUp")
  {
    set<string> entities = propOnto.getUp(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else if(req.action == "getDisjoint")
  {
    set<string> entities = propOnto.getDisjoint(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else if(req.action == "getInverse")
  {
    set<string> entities = propOnto.getInverse(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else if(req.action == "getDomain")
  {
    set<string> entities = propOnto.getDomain(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
  }
  else if(req.action == "getRange")
  {
    set<string> entities = propOnto.getRange(req.param);
    string result = "";
    for(set<string>::iterator it = entities.begin(); it != entities.end(); ++it)
      result += *it + " ";
    res.value = result;
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
    Ontology_reader reader(&onto, &propOnto);
    reader.readFile(string(argv[i]));
  }

  // Start up ROS service with callbacks
  ros::ServiceServer service = n.advertiseService("ontoloGenius/actions", reference_handle);
  ros::ServiceServer serviceClass = n.advertiseService("ontoloGenius/class", class_handle);
  ros::ServiceServer serviceProperty = n.advertiseService("ontoloGenius/property", property_handle);
  ROS_DEBUG("ontoloGenius ready");

  ros::spin();

  ROS_DEBUG("KILL ontoloGenius");

  return 0;
}
