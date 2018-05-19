#include "ontoloGenius/ontoGraphs/Ontology.h"
#include "ontoloGenius/arguer/Arguers.h"

#include "ontologenius/standard_service.h"
#include "ontologenius/ontologeniusService.h"

#include "ontoloGenius/utility/error_code.h"

#include <iostream>
#include "ros/ros.h"

#include "ontoloGenius/Computer.h"
#include "ontoloGenius/Parser.h"

using namespace std;

void removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.size() != 0))
    text.erase(0,1);

  while((text[text.size() - 1] == ' ') && (text.size() != 0))
    text.erase(text.size() - 1,1);
}

std::string set2string(std::set<std::string> word_set)
{
  string result = "";
  for(set<string>::iterator it = word_set.begin(); it != word_set.end(); ++it)
    result += *it + " ";
  return result;
}

Ontology onto;
Arguers arguers(&onto);


//DEPRECATED
bool reference_handle(ontologenius::standard_service::Request  &req,
                      ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

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
    if(comp.compute(req.param, onto.class_graph_))
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

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.class_graph_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.class_graph_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.class_graph_.getDisjoint(req.param));
    else if(req.action == "getName")
      res.value = onto.class_graph_.getName(req.param);
    else
      res.code = UNKNOW_ACTION;

  return true;
}

bool object_property_handle(ontologenius::standard_service::Request  &req,
                            ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.object_property_graph_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.object_property_graph_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.object_property_graph_.getDisjoint(req.param));
    else if(req.action == "getInverse")
      res.value = set2string(onto.object_property_graph_.getInverse(req.param));
    else if(req.action == "getDomain")
      res.value = set2string(onto.object_property_graph_.getDomain(req.param));
    else if(req.action == "getRange")
      res.value = set2string(onto.object_property_graph_.getRange(req.param));
    else if(req.action == "getName")
      res.value = onto.object_property_graph_.getName(req.param);
    else
      res.code = UNKNOW_ACTION;

  return true;
}

bool data_property_handle(ontologenius::standard_service::Request  &req,
                          ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.data_property_graph_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.data_property_graph_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.data_property_graph_.getDisjoint(req.param));
    else if(req.action == "getDomain")
      res.value = set2string(onto.data_property_graph_.getDomain(req.param));
    else if(req.action == "getRange")
      res.value = set2string(onto.data_property_graph_.getRange(req.param));
    else if(req.action == "getName")
      res.value = onto.data_property_graph_.getName(req.param);
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

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

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

      removeUselessSpace(select);
      removeUselessSpace(req.param);
    }
    removeUselessSpace(req.action);
  }

  if(res.code != UNINIT)
    if(req.action == "getSame")
      set_res = onto.individual_graph_.getSame(req.param);
    if(req.action == "getDistincts")
      set_res = onto.individual_graph_.getDistincts(req.param);
    else if(req.action == "getRelationFrom")
      set_res = onto.individual_graph_.getRelationFrom(req.param);
    else if(req.action == "getRelatedFrom")
      set_res = onto.individual_graph_.getRelatedFrom(req.param);
    else if(req.action == "getRelationOn")
      set_res = onto.individual_graph_.getRelationOn(req.param);
    else if(req.action == "getRelatedOn")
      set_res = onto.individual_graph_.getRelatedOn(req.param);
    else if(req.action == "getRelationWith")
      set_res = onto.individual_graph_.getRelationWith(req.param);
    else if(req.action == "getRelatedWith")
      set_res = onto.individual_graph_.getRelatedWith(req.param);
    else if(req.action == "getUp")
      set_res = onto.individual_graph_.getUp(req.param);
    else if(req.action == "getOn")
      set_res = onto.individual_graph_.getOn(req.param);
    else if(req.action == "getFrom")
      set_res = onto.individual_graph_.getFrom(req.param);
    else if(req.action == "getWith")
      set_res = onto.individual_graph_.getWith(req.param);
    else if(req.action == "getName")
      res.value = onto.individual_graph_.getName(req.param);
    else if(req.action == "find")
      set_res = onto.individual_graph_.find(req.param);
    else if(req.action == "getType")
      set_res = onto.individual_graph_.getType(req.param);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if(req.action == "getUp")
        set_res = onto.class_graph_.select(set_res, select);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith"))
        set_res = onto.object_property_graph_.select(set_res, select);
      else if(req.action != "getName")
        set_res = onto.individual_graph_.select(set_res, select);
    }

    if(res.value == "")
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
//END OF DEPRECATED

bool actionsHandle(ontologenius::ontologeniusService::Request  &req,
                      ontologenius::ontologeniusService::Response &res)
{
  bool done = false;
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

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
    if(comp.compute(req.param, onto.class_graph_))
      res.values[0] = "true";
    else
      res.values[0] = "false";
    //comp.compute("red_cube|young_animal=color_animal|age_object", onto);
  }
  else
    res.code = UNKNOW_ACTION;

  return true;
}

bool classHandle(ontologenius::standard_service::Request  &req,
                  ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.class_graph_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.class_graph_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.class_graph_.getDisjoint(req.param));
    else if(req.action == "getName")
      res.value = onto.class_graph_.getName(req.param);
    else
      res.code = UNKNOW_ACTION;

  return true;
}

bool objectPropertyHandle(ontologenius::standard_service::Request  &req,
                            ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.object_property_graph_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.object_property_graph_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.object_property_graph_.getDisjoint(req.param));
    else if(req.action == "getInverse")
      res.value = set2string(onto.object_property_graph_.getInverse(req.param));
    else if(req.action == "getDomain")
      res.value = set2string(onto.object_property_graph_.getDomain(req.param));
    else if(req.action == "getRange")
      res.value = set2string(onto.object_property_graph_.getRange(req.param));
    else if(req.action == "getName")
      res.value = onto.object_property_graph_.getName(req.param);
    else
      res.code = UNKNOW_ACTION;

  return true;
}

bool dataPropertyHandle(ontologenius::standard_service::Request  &req,
                          ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(res.code != UNINIT)
    if(req.action == "getDown")
      res.value = set2string(onto.data_property_graph_.getDown(req.param));
    else if(req.action == "getUp")
      res.value = set2string(onto.data_property_graph_.getUp(req.param));
    else if(req.action == "getDisjoint")
      res.value = set2string(onto.data_property_graph_.getDisjoint(req.param));
    else if(req.action == "getDomain")
      res.value = set2string(onto.data_property_graph_.getDomain(req.param));
    else if(req.action == "getRange")
      res.value = set2string(onto.data_property_graph_.getRange(req.param));
    else if(req.action == "getName")
      res.value = onto.data_property_graph_.getName(req.param);
    else
      res.code = UNKNOW_ACTION;

  return true;
}

bool individualHandle(ontologenius::standard_service::Request  &req,
                      ontologenius::standard_service::Response &res)
{
  bool done = false;
  res.value = "";
  res.code = 0;

  if(onto.isInit() == false)
    res.code = UNINIT;
  else
    arguers.runPreArguers();

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

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

      removeUselessSpace(select);
      removeUselessSpace(req.param);
    }
    removeUselessSpace(req.action);
  }

  if(res.code != UNINIT)
    if(req.action == "getSame")
      set_res = onto.individual_graph_.getSame(req.param);
    if(req.action == "getDistincts")
      set_res = onto.individual_graph_.getDistincts(req.param);
    else if(req.action == "getRelationFrom")
      set_res = onto.individual_graph_.getRelationFrom(req.param);
    else if(req.action == "getRelatedFrom")
      set_res = onto.individual_graph_.getRelatedFrom(req.param);
    else if(req.action == "getRelationOn")
      set_res = onto.individual_graph_.getRelationOn(req.param);
    else if(req.action == "getRelatedOn")
      set_res = onto.individual_graph_.getRelatedOn(req.param);
    else if(req.action == "getRelationWith")
      set_res = onto.individual_graph_.getRelationWith(req.param);
    else if(req.action == "getRelatedWith")
      set_res = onto.individual_graph_.getRelatedWith(req.param);
    else if(req.action == "getUp")
      set_res = onto.individual_graph_.getUp(req.param);
    else if(req.action == "getOn")
      set_res = onto.individual_graph_.getOn(req.param);
    else if(req.action == "getFrom")
      set_res = onto.individual_graph_.getFrom(req.param);
    else if(req.action == "getWith")
      set_res = onto.individual_graph_.getWith(req.param);
    else if(req.action == "getName")
      res.value = onto.individual_graph_.getName(req.param);
    else if(req.action == "find")
      set_res = onto.individual_graph_.find(req.param);
    else if(req.action == "getType")
      set_res = onto.individual_graph_.getType(req.param);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if(req.action == "getUp")
        set_res = onto.class_graph_.select(set_res, select);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith"))
        set_res = onto.object_property_graph_.select(set_res, select);
      else if(req.action != "getName")
        set_res = onto.individual_graph_.select(set_res, select);
    }

    if(res.value == "")
      res.value = set2string(set_res);

  return true;
}

bool arguerHandle(ontologenius::ontologeniusService::Request  &req,
                   ontologenius::ontologeniusService::Response &res)
{
  bool done = false;
  res.code = 0;

  if(req.action == "activate")
    res.code = arguers.activate(req.param);
  else if(req.action == "deactivate")
    res.code = arguers.deactivate(req.param);
  else if(req.action == "list")
    res.values = arguers.listVector();
  else if(req.action == "getDescription")
    res.values[0] = arguers.getDescription(req.param);
  else
    res.code = UNKNOW_ACTION;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius");

  ros::NodeHandle n;

  ros::service::waitForService("ontologenius/rest", -1);

  std::string language = string(argv[1]);
  std::cout << "language " << language << std::endl;

  for(unsigned int i = 2; i < argc; i++)
    onto.readFromFile(string(argv[i]));

  arguers.load();
  std::cout << "Plugins loaded : " << arguers.list() << std::endl;

  // Start up ROS service with callbacks

  //DEPRECATED
  ros::ServiceServer service_deprecated = n.advertiseService("ontoloGenius/actions", reference_handle);
  ros::ServiceServer serviceClass_deprecated = n.advertiseService("ontoloGenius/class", class_handle);
  ros::ServiceServer serviceObjectProperty_deprecated = n.advertiseService("ontoloGenius/object_property", object_property_handle);
  ros::ServiceServer serviceDataProperty_deprecated = n.advertiseService("ontoloGenius/data_property", data_property_handle);
  ros::ServiceServer serviceIndividual_deprecated = n.advertiseService("ontoloGenius/individual", individual_handle);
  ros::ServiceServer serviceArguer_deprecated = n.advertiseService("ontoloGenius/arguer", arguer_handle);
  //END OF DEPRECATED

  ros::ServiceServer service = n.advertiseService("ontologenius/actions", actionsHandle);
  ros::ServiceServer service_class = n.advertiseService("ontologenius/class", class_handle);
  ros::ServiceServer service_object_property = n.advertiseService("ontologenius/object_property", objectPropertyHandle);
  ros::ServiceServer service_data_property = n.advertiseService("ontologenius/data_property", dataPropertyHandle);
  ros::ServiceServer service_individual = n.advertiseService("ontologenius/individual", individualHandle);
  ros::ServiceServer service_arguer = n.advertiseService("ontologenius/arguer", arguerHandle);
  ROS_DEBUG("ontologenius ready");

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

  /*Error error;

  Code my_code(code);
  Parser p(&my_code);

  error.cpy(p.getError());
  error.printStatus();*/

  ros::spin();

  ROS_DEBUG("KILL ontoloGenius");

  return 0;
}
