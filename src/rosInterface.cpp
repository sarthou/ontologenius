#include "ontoloGenius/core/ontoGraphs/Ontology.h"
#include "ontoloGenius/core/arguer/Arguers.h"
#include "ontoloGenius/core/feeder/Feeder.h"
#include "ontoloGenius/core/utility/error_code.h"

#include "ontologenius/standard_service.h"
#include "ontologenius/OntologeniusService.h"
#include "std_msgs/String.h"

#include <iostream>
#include <thread>
#include "ros/ros.h"

#include "ontoloGenius/core/Computer.h"
#include "ontoloGenius/interpreter/Parser.h"

//#define USE_INTEPRETER

void removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.size() != 0))
    text.erase(0,1);

  while((text[text.size() - 1] == ' ') && (text.size() != 0))
    text.erase(text.size() - 1,1);
}

void set2string(const std::unordered_set<std::string>& word_set, std::string& result)
{
  for(const std::string& it : word_set)
    result += it + " ";
}

void set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result)
{
  for(const std::string& it : word_set)
    result.push_back(it);
}

int getPropagationLevel(std::string& params)
{
  size_t delimitater = params.find("<");
  if(delimitater != std::string::npos)
  {
    std::string param = params.substr(0, delimitater);
    std::string level = params.substr(delimitater+1);
    removeUselessSpace(level);
    params = param;

    int res;
    if(sscanf(level.c_str(), "%d", &res) != 1)
      res = -1;
    return res;
  }
  return -1;
}

std::string getSelector(std::string& action, std::string& param)
{
  std::string select = "";
  if(action.find("select:") == 0)
  {
    action = action.substr(std::string("select:").size());
    size_t delimitater = param.find("=");
    if(delimitater != std::string::npos)
    {
      select = param.substr(0, delimitater);
      param = param.substr(delimitater+1);

      removeUselessSpace(select);
      removeUselessSpace(param);
    }
    removeUselessSpace(action);
  }
  return select;
}

ros::NodeHandle* n_;
Ontology* onto;
Arguers arguers(onto);
Feeder feeder(onto);

bool actionsHandle(ontologenius::OntologeniusService::Request &req,
                   ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(req.action == "add")
    res.code = onto->readFromUri(req.param);
  else if(req.action == "fadd")
    res.code = onto->readFromFile(req.param);
  else if(req.action == "close")
  {
    onto->close();
    arguers.runPostArguers();
  }
  else if(req.action == "reset")
  {
    delete onto;
    onto = new Ontology();
    arguers.link(onto);
  }
  else if(req.action == "test")
  {
    Computer comp;
    res.values.resize(1);
    if(comp.compute(req.param, onto->class_graph_))
      res.values[0] = "true";
    else
      res.values[0] = "false";
    //comp.compute("red_cube|young_animal=color_animal|age_object", onto);
  }
  else if(req.action == "setLang")
    onto->setLanguage(req.param);
  else
    res.code = UNKNOW_ACTION;

  return true;
}

bool classHandle(ontologenius::OntologeniusService::Request &req,
                  ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto->isInit() == false)
    res.code = UNINIT;
  else
  {
    arguers.runPreArguers();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getDown")
      set_res = onto->class_graph_.getDown(req.param, level);
    else if(req.action == "getUp")
      set_res = onto->class_graph_.getUp(req.param, level);
    else if(req.action == "getDisjoint")
      set_res = onto->class_graph_.getDisjoint(req.param);
    else if(req.action == "getName")
      res.values.push_back(onto->class_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto->class_graph_.getNames(req.param);
    else if(req.action == "getRelationFrom")
      set_res = onto->class_graph_.getRelationFrom(req.param, level);
    else if(req.action == "getRelatedFrom")
      set_res = onto->class_graph_.getRelatedFrom(req.param);
    else if(req.action == "getRelationOn")
      set_res = onto->class_graph_.getRelationOn(req.param, level);
    else if(req.action == "getRelatedOn")
      set_res = onto->class_graph_.getRelatedOn(req.param);
    else if(req.action == "getRelationWith")
      set_res = onto->class_graph_.getRelationWith(req.param);
    else if(req.action == "getRelatedWith")
      set_res = onto->class_graph_.getRelatedWith(req.param);
    else if(req.action == "getUp")
      set_res = onto->class_graph_.getUp(req.param, level);
    else if(req.action == "getOn")
      set_res = onto->class_graph_.getOn(req.param);
    else if(req.action == "getFrom")
      set_res = onto->class_graph_.getFrom(req.param);
    else if(req.action == "getWith")
      set_res = onto->class_graph_.getWith(req.param, level);
    else if(req.action == "find")
      set2vector(onto->class_graph_.find(req.param), res.values);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") ||
        (req.action == "getDisjoint"))
        set_res = onto->class_graph_.select(set_res, select);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith"))
        set_res = onto->object_property_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool objectPropertyHandle(ontologenius::OntologeniusService::Request &req,
                            ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto->isInit() == false)
    res.code = UNINIT;
  else
  {
    arguers.runPreArguers();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getDown")
      set_res = onto->object_property_graph_.getDown(req.param, level);
    else if(req.action == "getUp")
      set_res = onto->object_property_graph_.getUp(req.param, level);
    else if(req.action == "getDisjoint")
      set_res = onto->object_property_graph_.getDisjoint(req.param);
    else if(req.action == "getInverse")
      set_res = onto->object_property_graph_.getInverse(req.param);
    else if(req.action == "getDomain")
      set_res = onto->object_property_graph_.getDomain(req.param);
    else if(req.action == "getRange")
      set_res = onto->object_property_graph_.getRange(req.param);
    else if(req.action == "getName")
      res.values.push_back(onto->object_property_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto->object_property_graph_.getNames(req.param);
    else if(req.action == "find")
      set2vector(onto->object_property_graph_.find(req.param), res.values);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") ||
        (req.action == "getDisjoint") || (req.action == "getInverse"))
        set_res = onto->object_property_graph_.select(set_res, select);
      else if((req.action == "getDomain") || (req.action == "getRange"))
        set_res = onto->class_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool dataPropertyHandle(ontologenius::OntologeniusService::Request &req,
                          ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto->isInit() == false)
    res.code = UNINIT;
  else
  {
    arguers.runPreArguers();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getDown")
      set_res = onto->data_property_graph_.getDown(req.param, level);
    else if(req.action == "getUp")
      set_res = onto->data_property_graph_.getUp(req.param, level);
    else if(req.action == "getDisjoint")
      set_res = onto->data_property_graph_.getDisjoint(req.param);
    else if(req.action == "getDomain")
      set_res = onto->data_property_graph_.getDomain(req.param);
    else if(req.action == "getRange")
      set2vector(onto->data_property_graph_.getRange(req.param), res.values);
    else if(req.action == "getName")
      res.values.push_back(onto->data_property_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto->data_property_graph_.getNames(req.param);
    else if(req.action == "find")
      set2vector(onto->data_property_graph_.find(req.param), res.values);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") || (req.action == "getDisjoint"))
        set_res = onto->data_property_graph_.select(set_res, select);
      else if(req.action == "getDomain")
        set_res = onto->class_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool individualHandle(ontologenius::OntologeniusService::Request  &req,
                      ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto->isInit() == false)
    res.code = UNINIT;
  else
  {
    arguers.runPreArguers();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getSame")
      set_res = onto->individual_graph_.getSame(req.param);
    if(req.action == "getDistincts")
      set_res = onto->individual_graph_.getDistincts(req.param);
    else if(req.action == "getRelationFrom")
      set_res = onto->individual_graph_.getRelationFrom(req.param, level);
    else if(req.action == "getRelatedFrom")
      set_res = onto->individual_graph_.getRelatedFrom(req.param);
    else if(req.action == "getRelationOn")
      set_res = onto->individual_graph_.getRelationOn(req.param, level);
    else if(req.action == "getRelatedOn")
      set_res = onto->individual_graph_.getRelatedOn(req.param);
    else if(req.action == "getRelationWith")
      set_res = onto->individual_graph_.getRelationWith(req.param);
    else if(req.action == "getRelatedWith")
      set_res = onto->individual_graph_.getRelatedWith(req.param);
    else if(req.action == "getUp")
      set_res = onto->individual_graph_.getUp(req.param, level);
    else if(req.action == "getOn")
      set_res = onto->individual_graph_.getOn(req.param);
    else if(req.action == "getFrom")
      set_res = onto->individual_graph_.getFrom(req.param);
    else if(req.action == "getWith")
      set_res = onto->individual_graph_.getWith(req.param, level);
    else if(req.action == "getName")
      res.values.push_back(onto->individual_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto->individual_graph_.getNames(req.param);
    else if(req.action == "find")
      set_res = onto->individual_graph_.find(req.param);
    else if(req.action == "getType")
      set_res = onto->individual_graph_.getType(req.param);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if(req.action == "getUp")
        set_res = onto->class_graph_.select(set_res, select);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith"))
        set_res = onto->object_property_graph_.select(set_res, select);
      else if(req.action != "find")
        set_res = onto->individual_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool arguerHandle(ontologenius::OntologeniusService::Request &req,
                   ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(req.action == "activate")
    res.code = arguers.activate(req.param);
  else if(req.action == "deactivate")
    res.code = arguers.deactivate(req.param);
  else if(req.action == "list")
    res.values = arguers.listVector();
  else if(req.action == "getDescription")
    res.values.push_back(arguers.getDescription(req.param));
  else
    res.code = UNKNOW_ACTION;

  return true;
}

void knowledgeCallback(const std_msgs::String::ConstPtr& msg)
{
  feeder.store(msg->data);
}

void feedThread()
{
  ros::Rate wait(10);
  while((ros::ok()) && (onto->isInit(false) == false))
  {
    wait.sleep();
  }

  while(ros::ok())
  {
    feeder.run();
    wait.sleep();
  }
}

void periodicReasoning()
{
  ros::Publisher arguer_publisher = n_->advertise<std_msgs::String>("ontologenius/arguer_notifications", 1000);

  ros::Rate wait(10);
  while((ros::ok()) && (onto->isInit(false) == false))
  {
    wait.sleep();
  }

  ros::Rate r(100);
  while(ros::ok())
  {
    arguers.runPeriodicArguers();

    std_msgs::String msg;
    std::vector<std::string> notifications = arguers.getNotifications();
    for(auto notif : notifications)
    {
      std::cout << notif << std::endl;
      msg.data = notif;
      arguer_publisher.publish(msg);
    }
    r.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius");

  ros::NodeHandle n;
  n_ = &n;

  ros::service::waitForService("ontologenius/rest", -1);

  onto = new Ontology();
  arguers.link(onto);
  feeder.link(onto);

  std::string language = std::string(argv[1]);
  std::cout << "language " << language << std::endl;

  std::string intern_file = std::string(argv[2]);
  std::cout << "intern_file " << intern_file << std::endl;

  if(onto->preload(intern_file) == false)
    for(int i = 3; i < argc; i++)
      onto->readFromFile(std::string(argv[i]));

  arguers.load();
  std::cout << "Plugins loaded : " << arguers.list() << std::endl;

  ros::Subscriber knowledge_subscriber = n.subscribe("ontologenius/insert", 1000, knowledgeCallback);

  // Start up ROS service with callbacks
  ros::ServiceServer service = n.advertiseService("ontologenius/actions", actionsHandle);
  ros::ServiceServer service_class = n.advertiseService("ontologenius/class", classHandle);
  ros::ServiceServer service_object_property = n.advertiseService("ontologenius/object_property", objectPropertyHandle);
  ros::ServiceServer service_data_property = n.advertiseService("ontologenius/data_property", dataPropertyHandle);
  ros::ServiceServer service_individual = n.advertiseService("ontologenius/individual", individualHandle);
  ros::ServiceServer service_arguer = n.advertiseService("ontologenius/arguer", arguerHandle);
  ROS_DEBUG("ontologenius ready");

#ifdef USE_INTEPRETER
  std::string code = "";
  code += "var::man += fablab.isIn() - (bob + max);\n";
  code += "var::man.toString();\n";
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
#endif

  std::thread periodic_reasoning_thread(periodicReasoning);
  std::thread feed_thread(feedThread);

  ros::spin();
  periodic_reasoning_thread.join();
  feed_thread.join();

  delete onto;

  ROS_DEBUG("KILL ontoloGenius");

  return 0;
}
