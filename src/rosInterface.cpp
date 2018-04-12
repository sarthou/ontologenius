#include "ontoloGenius/ontoGraphs/Ontology.h"
#include "ontoloGenius/arguer/Arguers.h"

#include "ontologenius/standard_service.h"
#include "ontoloGenius/utility/error_code.h"

#include <iostream>
#include "ros/ros.h"

#include "ontoloGenius/Computer.h"

#include "ontoloGenius/Parser.h"

using namespace std;

std::string set2string(std::set<std::string> word_set)
{
  string result = "";
  for(set<string>::iterator it = word_set.begin(); it != word_set.end(); ++it)
    result += *it + " ";
  return result;
}

Ontology onto;
Arguers arguers(&onto);

bool reference_handle(ontologenius::standard_service::Request  &req,
                      ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(req.action == "add")
    res.code = onto.readFromUri(req.param);
  else if(req.action == "close")
  {
    onto.close();
    arguers.runPostArguers();
  }
  else if(req.action == "reset")
    onto = Ontology();
  else if(req.action == "test")
  {
    Computer comp;
    if(comp.compute(req.param, onto.classes_))
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

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.classes_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.classes_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.classes_.getDisjoint(req.param));
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

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.properties_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.properties_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.properties_.getDisjoint(req.param));
    else if(req.action == "getInverse")
      res.value = set2string(onto.properties_.getInverse(req.param));
    else if(req.action == "getDomain")
      res.value = set2string(onto.properties_.getDomain(req.param));
    else if(req.action == "getRange")
      res.value = set2string(onto.properties_.getRange(req.param));
    else
      res.code = UNKNOW_ACTION;

  return true;
}

bool individual_handle(ontologenius::standard_service::Request  &req,
                      ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  std::set<std::string> set_res;
  std::string select = "";
  if(req.action.find("select:") == 0)
  {
    req.action = req.action.substr(std::string("select:").size());
    size_t delimitater = req.param.find("=");
    if(delimitater != std::string::npos)
    {
      select = req.param.substr(0, delimitater);
      req.param = req.param.substr(delimitater+1);
    }
  }

  if(res.code != UNINIT)
    if(req.action == "getSame")
      set_res = onto.individuals_.getSame(req.param);
    if(req.action == "getDistincts")
      set_res = onto.individuals_.getDistincts(req.param);
    else if(req.action == "getRelationFrom")
      set_res = onto.individuals_.getRelationFrom(req.param);
    else if(req.action == "getRelatedFrom")
      set_res = onto.individuals_.getRelatedFrom(req.param);
    else if(req.action == "getRelationOn")
      set_res = onto.individuals_.getRelationOn(req.param);
    else if(req.action == "getRelatedOn")
      set_res = onto.individuals_.getRelatedOn(req.param);
    else if(req.action == "getRelationWith")
      set_res = onto.individuals_.getRelationWith(req.param);
    else if(req.action == "getRelatedWith")
      set_res = onto.individuals_.getRelatedWith(req.param);
    else if(req.action == "getUp")
      set_res = onto.individuals_.getUp(req.param);
    else if(req.action == "getOn")
      set_res = onto.individuals_.getOn(req.param);
    else if(req.action == "getFrom")
      set_res = onto.individuals_.getFrom(req.param);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if(req.action == "getUp")
        set_res = onto.classes_.selectOnClass(set_res, select);
      else
        set_res = onto.individuals_.selectOnClass(set_res, select);
    }

    res.value = set2string(set_res);

  return true;
}

bool arguer_handle(ontologenius::standard_service::Request  &req,
                   ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(req.action == "activate")
    res.code = arguers.activate(req.param);
  else if(req.action == "deactivate")
    res.code = arguers.deactivate(req.param);
  else if(req.action == "list")
    res.value = arguers.list();
  else if(req.action == "getDescription")
    res.value = arguers.getDescription(req.param);
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
    onto.readFromFile(string(argv[i]));

  arguers.load();
  arguers.list();

  // Start up ROS service with callbacks
  ros::ServiceServer service = n.advertiseService("ontoloGenius/actions", reference_handle);
  ros::ServiceServer serviceClass = n.advertiseService("ontoloGenius/class", class_handle);
  ros::ServiceServer serviceProperty = n.advertiseService("ontoloGenius/property", property_handle);
  ros::ServiceServer serviceIndividual = n.advertiseService("ontoloGenius/individual", individual_handle);
  ros::ServiceServer serviceArguer = n.advertiseService("ontoloGenius/arguer", arguer_handle);
  ROS_DEBUG("ontoloGenius ready");

  std::string code = "";
  code += "var::man += fablab.isIn() - (bob + max);\n";
  code += "var::man.toString(test, test);\n";
  code += "if(adult == age) \n";
  code += "{";
  code += "//this is a comment\n";
  code += "\tif(age == adult)\n";
  code += "\t\tont::print(var::man);\n";
  code += "}\n";
  code += "else if(old == /* */age)\n";
  code += "\tif(young == age)\n";
  code += "\t\tont::print(\"this is an else if\");\n";
  code += "/*\n";
  code += "an other comment*/ \n";
  code += "ont::print(\"this is the else \");\n\n";
  code += "ont::null();\n";
  code += "var::men =if(var::man == man);\n";

  Error error;

  Code my_code(code);
  Parser p(&my_code);

  error.cpy(p.getError());
  error.printStatus();

  ros::spin();

  ROS_DEBUG("KILL ontoloGenius");

  return 0;
}
