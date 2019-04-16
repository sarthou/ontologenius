#include <thread>

#include <ros/callback_queue.h>

#include "ontoloGenius/RosInterface.h"

#include "ontoloGenius/core/utility/error_code.h"

namespace ontologenius {

RosInterface::RosInterface(ros::NodeHandle* n, std::string name) : reasoners_(onto_), feeder_(onto_), run_(true),
                                                                  pub_(n->advertise<std_msgs::String>("ontologenius/end", 1000))
{
  n_ = n;
  onto_ = new Ontology();
  reasoners_.link(onto_);
  feeder_.link(onto_);

  name_ = name;
  feeder_end = true;
}

RosInterface::~RosInterface()
{
  delete onto_;
}

void RosInterface::init(std::string& lang, std::string intern_file, std::vector<std::string>& files)
{
  onto_->setLanguage(lang);
  std::string ontology_intern_file = intern_file;
  if(name_ != "")
  {
    size_t pose = intern_file.find(".owl");
    if(pose != std::string::npos)
      intern_file.insert(pose, "_" + name_);
  }

  if(onto_->preload(intern_file) == false)
    for(auto file : files)
      onto_->readFromFile(file);

  reasoners_.load();
  std::cout << "Plugins loaded : " << reasoners_.list() << std::endl;

}

void RosInterface::run()
{
  std::string service_name;

  service_name = (name_ == "") ? "ontologenius/insert" : "ontologenius/insert/" + name_;
  ros::Subscriber knowledge_subscriber = n_->subscribe(service_name, 1000, &RosInterface::knowledgeCallback, this);

  // Start up ROS service with callbacks
  service_name = (name_ == "") ? "ontologenius/actions" : "ontologenius/actions/" + name_;
  ros::ServiceServer service = n_->advertiseService(service_name, &RosInterface::actionsHandle, this);

  service_name = (name_ == "") ? "ontologenius/class" : "ontologenius/class/" + name_;
  ros::ServiceServer service_class = n_->advertiseService(service_name, &RosInterface::classHandle, this);

  service_name = (name_ == "") ? "ontologenius/object_property" : "ontologenius/object_property/" + name_;
  ros::ServiceServer service_object_property = n_->advertiseService(service_name, &RosInterface::objectPropertyHandle, this);

  service_name = (name_ == "") ? "ontologenius/data_property" : "ontologenius/data_property/" + name_;
  ros::ServiceServer service_data_property = n_->advertiseService(service_name, &RosInterface::dataPropertyHandle, this);

  service_name = (name_ == "") ? "ontologenius/individual" : "ontologenius/individual/" + name_;
  ros::ServiceServer service_individual = n_->advertiseService(service_name, &RosInterface::individualHandle, this);

  service_name = (name_ == "") ? "ontologenius/reasoner" : "ontologenius/reasoner/" + name_;
  ros::ServiceServer service_reasoner = n_->advertiseService(service_name, &RosInterface::reasonerHandle, this);


  std::thread feed_thread(&RosInterface::feedThread, this);
  std::thread periodic_reasoning_thread(&RosInterface::periodicReasoning, this);

  ROS_DEBUG("%s ontologenius ready", name_.c_str());

  while (ros::ok() && isRunning())
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  periodic_reasoning_thread.join();
  feed_thread.join();
}


/***************
*
* Callbacks
*
****************/

void RosInterface::knowledgeCallback(const std_msgs::String::ConstPtr& msg)
{
  feeder_.store(msg->data);
}

bool RosInterface::actionsHandle(ontologenius::OntologeniusService::Request &req,
                                 ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(req.action == "add")
    res.code = onto_->readFromUri(req.param);
  else if(req.action == "fadd")
    res.code = onto_->readFromFile(req.param);
  else if(req.action == "close")
  {
    onto_->close();
    reasoners_.runPostReasoners();
  }
  else if(req.action == "reset")
  {
    feeder_mutex_.lock();
    reasoner_mutex_.lock();
    delete onto_;
    onto_ = new Ontology();
    reasoners_.link(onto_);
    feeder_.link(onto_);
    feeder_mutex_.unlock();
    reasoner_mutex_.unlock();
  }
  else if(req.action == "setLang")
    onto_->setLanguage(req.param);
  else if(req.action == "getLang")
    res.values.push_back(onto_->getLanguage());
  else
    res.code = UNKNOW_ACTION;

  return true;
}

bool RosInterface::classHandle(ontologenius::OntologeniusService::Request &req,
                               ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto_->isInit() == false)
    res.code = UNINIT;
  else
  {
    reasoners_.runPreReasoners();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getDown")
      set_res = onto_->class_graph_.getDown(req.param, level);
    else if(req.action == "getUp")
      set_res = onto_->class_graph_.getUp(req.param, level);
    else if(req.action == "getDisjoint")
      set_res = onto_->class_graph_.getDisjoint(req.param);
    else if(req.action == "getName")
      res.values.push_back(onto_->class_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto_->class_graph_.getNames(req.param);
    else if(req.action == "getRelationFrom")
      set_res = onto_->class_graph_.getRelationFrom(req.param, level);
    else if(req.action == "getRelatedFrom")
      set_res = onto_->class_graph_.getRelatedFrom(req.param);
    else if(req.action == "getRelationOn")
      set_res = onto_->class_graph_.getRelationOn(req.param, level);
    else if(req.action == "getRelatedOn")
      set_res = onto_->class_graph_.getRelatedOn(req.param);
    else if(req.action == "getRelationWith")
      set_res = onto_->class_graph_.getRelationWith(req.param);
    else if(req.action == "getRelatedWith")
      set_res = onto_->class_graph_.getRelatedWith(req.param);
    else if(req.action == "getUp")
      set_res = onto_->class_graph_.getUp(req.param, level);
    else if(req.action == "getOn")
      set_res = onto_->class_graph_.getOn(req.param);
    else if(req.action == "getFrom")
      set_res = onto_->class_graph_.getFrom(req.param);
    else if(req.action == "getWith")
      set_res = onto_->class_graph_.getWith(req.param, level);
    else if(req.action == "find")
      set2vector(onto_->class_graph_.find(req.param), res.values);
    else if(req.action == "findRegex")
      set2vector(onto_->class_graph_.find(req.param), res.values);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") ||
        (req.action == "getDisjoint") || (req.action == "getOn") ||
        (req.action == "getFrom"))
        set_res = onto_->class_graph_.select(set_res, select);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith"))
        set_res = onto_->object_property_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool RosInterface::objectPropertyHandle(ontologenius::OntologeniusService::Request &req,
                                        ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto_->isInit() == false)
    res.code = UNINIT;
  else
  {
    reasoners_.runPreReasoners();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getDown")
      set_res = onto_->object_property_graph_.getDown(req.param, level);
    else if(req.action == "getUp")
      set_res = onto_->object_property_graph_.getUp(req.param, level);
    else if(req.action == "getDisjoint")
      set_res = onto_->object_property_graph_.getDisjoint(req.param);
    else if(req.action == "getInverse")
      set_res = onto_->object_property_graph_.getInverse(req.param);
    else if(req.action == "getDomain")
      set_res = onto_->object_property_graph_.getDomain(req.param);
    else if(req.action == "getRange")
      set_res = onto_->object_property_graph_.getRange(req.param);
    else if(req.action == "getName")
      res.values.push_back(onto_->object_property_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto_->object_property_graph_.getNames(req.param);
    else if(req.action == "find")
      set2vector(onto_->object_property_graph_.find(req.param), res.values);
    else if(req.action == "findRegex")
      set2vector(onto_->object_property_graph_.find(req.param), res.values);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") ||
        (req.action == "getDisjoint") || (req.action == "getInverse"))
        set_res = onto_->object_property_graph_.select(set_res, select);
      else if((req.action == "getDomain") || (req.action == "getRange"))
        set_res = onto_->class_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool RosInterface::dataPropertyHandle(ontologenius::OntologeniusService::Request &req,
                                      ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto_->isInit() == false)
    res.code = UNINIT;
  else
  {
    reasoners_.runPreReasoners();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getDown")
      set_res = onto_->data_property_graph_.getDown(req.param, level);
    else if(req.action == "getUp")
      set_res = onto_->data_property_graph_.getUp(req.param, level);
    else if(req.action == "getDisjoint")
      set_res = onto_->data_property_graph_.getDisjoint(req.param);
    else if(req.action == "getDomain")
      set_res = onto_->data_property_graph_.getDomain(req.param);
    else if(req.action == "getRange")
      set2vector(onto_->data_property_graph_.getRange(req.param), res.values);
    else if(req.action == "getName")
      res.values.push_back(onto_->data_property_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto_->data_property_graph_.getNames(req.param);
    else if(req.action == "find")
      set2vector(onto_->data_property_graph_.find(req.param), res.values);
    else if(req.action == "findRegex")
      set2vector(onto_->data_property_graph_.find(req.param), res.values);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") || (req.action == "getDisjoint"))
        set_res = onto_->data_property_graph_.select(set_res, select);
      else if(req.action == "getDomain")
        set_res = onto_->class_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool RosInterface::individualHandle(ontologenius::OntologeniusService::Request  &req,
                                    ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(onto_->isInit() == false)
    res.code = UNINIT;
  else
  {
    reasoners_.runPreReasoners();

    int level = getPropagationLevel(req.param);

    removeUselessSpace(req.action);
    removeUselessSpace(req.param);

    std::unordered_set<std::string> set_res;
    std::string select = getSelector(req.action, req.param);

    if(req.action == "getSame")
      set_res = onto_->individual_graph_.getSame(req.param);
    if(req.action == "getDistincts")
      set_res = onto_->individual_graph_.getDistincts(req.param);
    else if(req.action == "getRelationFrom")
      set_res = onto_->individual_graph_.getRelationFrom(req.param, level);
    else if(req.action == "getRelatedFrom")
      set_res = onto_->individual_graph_.getRelatedFrom(req.param);
    else if(req.action == "getRelationOn")
      set_res = onto_->individual_graph_.getRelationOn(req.param, level);
    else if(req.action == "getRelatedOn")
      set_res = onto_->individual_graph_.getRelatedOn(req.param);
    else if(req.action == "getRelationWith")
      set_res = onto_->individual_graph_.getRelationWith(req.param);
    else if(req.action == "getRelatedWith")
      set_res = onto_->individual_graph_.getRelatedWith(req.param);
    else if(req.action == "getUp")
      set_res = onto_->individual_graph_.getUp(req.param, level);
    else if(req.action == "getOn")
      set_res = onto_->individual_graph_.getOn(req.param);
    else if(req.action == "getFrom")
      set_res = onto_->individual_graph_.getFrom(req.param);
    else if(req.action == "getWith")
      set_res = onto_->individual_graph_.getWith(req.param, level);
    else if(req.action == "getName")
      res.values.push_back(onto_->individual_graph_.getName(req.param));
    else if(req.action == "getNames")
      res.values = onto_->individual_graph_.getNames(req.param);
    else if(req.action == "find")
      set_res = onto_->individual_graph_.find(req.param);
    else if(req.action == "findRegex")
      set_res = onto_->individual_graph_.find(req.param);
    else if(req.action == "getType")
      set_res = onto_->individual_graph_.getType(req.param);
    else
      res.code = UNKNOW_ACTION;

    if(select != "")
    {
      if(req.action == "getUp")
        set_res = onto_->class_graph_.select(set_res, select);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith"))
        set_res = onto_->object_property_graph_.select(set_res, select);
      else if((req.action != "find") || (req.action != "findRegex"))
        set_res = onto_->individual_graph_.select(set_res, select);
    }

    if(res.values.size() == 0)
      set2vector(set_res, res.values);
  }

  return true;
}

bool RosInterface::reasonerHandle(ontologenius::OntologeniusService::Request &req,
                                ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  if(req.action == "activate")
    res.code = reasoners_.activate(req.param);
  else if(req.action == "deactivate")
    res.code = reasoners_.deactivate(req.param);
  else if(req.action == "list")
    res.values = reasoners_.listVector();
  else if(req.action == "getDescription")
    res.values.push_back(reasoners_.getDescription(req.param));
  else
    res.code = UNKNOW_ACTION;

  return true;
}

/***************
*
* Threads
*
***************/

void RosInterface::feedThread()
{
  std::string publisher_name = (name_ == "") ? "ontologenius/feeder_notifications" : "ontologenius/feeder_notifications/" + name_;
  ros::Publisher feeder_publisher = n_->advertise<std_msgs::String>(publisher_name, 1000);

  ros::Rate wait(10);
  while((ros::ok()) && (onto_->isInit(false) == false) && (run_ == true))
  {
    wait.sleep();
  }

  while(ros::ok() && (run_ == true))
  {
    feeder_mutex_.lock();
    bool run = feeder_.run();
    if(run == true)
    {
      if(feeder_end == true)
        feeder_end = false;

      reasoners_.runPostReasoners();

      std_msgs::String msg;
      std::vector<std::string> notifications = feeder_.getNotifications();
      for(auto notif : notifications)
      {
        std::cout << notif << std::endl;
        msg.data = notif;
        feeder_publisher.publish(msg);
      }
    }
    else if(feeder_end == false)
    {
      feeder_end = true;
      std_msgs::String msg;
      msg.data = "end";
      pub_.publish(msg);
    }
    feeder_mutex_.unlock();
    wait.sleep();
  }
}

void RosInterface::periodicReasoning()
{
  std::string publisher_name = (name_ == "") ? "ontologenius/reasoner_notifications" : "ontologenius/reasoner_notifications/" + name_;
  ros::Publisher reasoner_publisher = n_->advertise<std_msgs::String>(publisher_name, 1000);

  ros::Rate wait(10);
  while((ros::ok()) && (onto_->isInit(false) == false) && (run_ == true))
  {
    wait.sleep();
  }

  ros::Rate r(100);
  while(ros::ok() && (run_ == true))
  {
    reasoner_mutex_.lock();
    reasoners_.runPeriodicReasoners();

    std_msgs::String msg;
    std::vector<std::string> notifications = reasoners_.getNotifications();
    for(auto notif : notifications)
    {
      std::cout << notif << std::endl;
      msg.data = notif;
      reasoner_publisher.publish(msg);
    }
    reasoner_mutex_.unlock();
    r.sleep();
  }
}

/***************
*
* Utility
*
****************/

void RosInterface::removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.size() != 0))
    text.erase(0,1);

  while((text[text.size() - 1] == ' ') && (text.size() != 0))
    text.erase(text.size() - 1,1);
}

void RosInterface::set2string(const std::unordered_set<std::string>& word_set, std::string& result)
{
  for(const std::string& it : word_set)
    result += it + " ";
}

void RosInterface::set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result)
{
  for(const std::string& it : word_set)
    result.push_back(it);
}

int RosInterface::getPropagationLevel(std::string& params)
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

std::string RosInterface::getSelector(std::string& action, std::string& param)
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

} // namespace ontologenius
