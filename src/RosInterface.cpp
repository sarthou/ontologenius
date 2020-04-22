#include <thread>

#include <ros/callback_queue.h>

#include "ontologenius/RosInterface.h"

#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"

#define PUB_QUEU_SIZE 1000
#define SUB_QUEU_SIZE 10000

namespace ontologenius {

RosInterface::RosInterface(ros::NodeHandle* n, const std::string& name) : run_(true),
                                                                   pub_(n->advertise<std_msgs::String>(name == "" ? "ontologenius/end" : "ontologenius/end/" + name, PUB_QUEU_SIZE))
{
  n_ = n;
  onto_ = new Ontology();
  reasoners_.link(onto_);
  feeder_.link(onto_);

  name_ = name;
  feeder_end = true;
}

RosInterface::RosInterface(RosInterface& other, ros::NodeHandle* n, const std::string& name) : run_(true),
                                                                   pub_(n->advertise<std_msgs::String>(name == "" ? "ontologenius/end" : "ontologenius/end/" + name, PUB_QUEU_SIZE))
{
  n_ = n;

  other.lock();
  onto_ = new Ontology(*other.onto_);
  other.release();

  reasoners_.link(onto_);
  feeder_.link(onto_);

  name_ = name;
  feeder_end = true;
}

RosInterface::~RosInterface()
{
  delete onto_;
}

void RosInterface::init(const std::string& lang, const std::string& intern_file, const std::vector<std::string>& files, const std::string& config_path)
{
  onto_->setLanguage(lang);
  std::string dedicated_intern_file = intern_file;
  if(name_ != "")
  {
    size_t pose = dedicated_intern_file.find(".owl");
    if(pose != std::string::npos)
      dedicated_intern_file.insert(pose, "_" + name_);
  }

  if(onto_->preload(dedicated_intern_file) == false)
    for(auto& file : files)
      onto_->readFromFile(file);

  reasoners_.configure(config_path);
  reasoners_.load();
  Display::info("Plugins loaded : " + reasoners_.list());

  feeder_rate_ = 20;
}

void RosInterface::init(const std::string& lang, const std::string& config_path)
{
  onto_->setLanguage(lang);

  reasoners_.configure(config_path);
  reasoners_.load();
  Display::info("Plugins loaded : " + reasoners_.list());

  feeder_.activateVersionning(true);
  feeder_rate_ = 4000;
}

void RosInterface::run()
{
  ros::Subscriber knowledge_subscriber = n_->subscribe(getTopicName("insert"), PUB_QUEU_SIZE, &RosInterface::knowledgeCallback, this);

  ros::Subscriber stamped_knowledge_subscriber = n_->subscribe(getTopicName("insert_stamped"), PUB_QUEU_SIZE, &RosInterface::stampedKnowledgeCallback, this);

  // Start up ROS service with callbacks
  ros::ServiceServer service = n_->advertiseService(getTopicName("actions"), &RosInterface::actionsHandle, this);

  ros::ServiceServer service_class = n_->advertiseService(getTopicName("class"), &RosInterface::classHandle, this);

  ros::ServiceServer service_object_property = n_->advertiseService(getTopicName("object_property"), &RosInterface::objectPropertyHandle, this);

  ros::ServiceServer service_data_property = n_->advertiseService(getTopicName("data_property"), &RosInterface::dataPropertyHandle, this);

  ros::ServiceServer service_individual = n_->advertiseService(getTopicName("individual"), &RosInterface::individualHandle, this);

  ros::ServiceServer service_reasoner = n_->advertiseService(getTopicName("reasoner"), &RosInterface::reasonerHandle, this);


  std::thread feed_thread(&RosInterface::feedThread, this);
  std::thread periodic_reasoning_thread(&RosInterface::periodicReasoning, this);

  Display::info(name_ + " is ready");

  while (ros::ok() && isRunning())
  {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  periodic_reasoning_thread.join();
  feed_thread.join();
}

void RosInterface::lock()
{
  feeder_mutex_.lock();
  reasoner_mutex_.lock();
}

void RosInterface::release()
{
  feeder_mutex_.unlock();
  reasoner_mutex_.unlock();
}

void RosInterface::close()
{
  onto_->close();
  reasoners_.runPostReasoners();
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

void RosInterface::stampedKnowledgeCallback(const ontologenius::StampedString::ConstPtr& msg)
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
  else if(req.action == "save")
    onto_->save(req.param);
  else if(req.action == "export")
    feeder_.exportToXml(req.param);
  else if(req.action == "close")
    close();
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
    removeUselessSpace(req.action);
    param_t params = getParams(req.param);

    //Remove for V2.4
    int level = getPropagationLevel(params.base);
    if(level != -1)
      params.depth = level;
    std::string select = getSelector(req.action, params.base);
    if(select != "")
      params.selector = select;
    // end remove for 2.4

    std::unordered_set<std::string> set_res;

    if(req.action == "getDown")
      set_res = onto_->class_graph_.getDown(params(), params.depth);
    else if(req.action == "getUp")
      set_res = onto_->class_graph_.getUp(params(), params.depth);
    else if(req.action == "getDisjoint")
      set_res = onto_->class_graph_.getDisjoint(params());
    else if(req.action == "getName")
    {
      auto tmp = onto_->class_graph_.getName(params(), params.take_id);
      if(tmp != "")
        res.values.push_back(tmp);
    }
    else if(req.action == "getNames")
      res.values = onto_->class_graph_.getNames(params());
    else if(req.action == "getEveryNames")
      res.values = onto_->class_graph_.getEveryNames(params());
    else if(req.action == "getRelationFrom")
      set_res = onto_->class_graph_.getRelationFrom(params(), params.depth);
    else if(req.action == "getRelatedFrom")
      set_res = onto_->class_graph_.getRelatedFrom(params());
    else if(req.action == "getRelationOn")
      set_res = onto_->class_graph_.getRelationOn(params(), params.depth);
    else if(req.action == "getRelatedOn")
      set_res = onto_->class_graph_.getRelatedOn(params());
    else if(req.action == "getRelationWith")
      set_res = onto_->class_graph_.getRelationWith(params());
    else if(req.action == "getRelatedWith")
      set_res = onto_->class_graph_.getRelatedWith(params());
    else if(req.action == "getOn")
      set_res = onto_->class_graph_.getOn(params());
    else if(req.action == "getFrom")
      set_res = onto_->class_graph_.getFrom(params());
    else if(req.action == "getWith")
      set_res = onto_->class_graph_.getWith(params(), params.depth);
    else if(req.action == "getDomainOf")
      set_res = onto_->class_graph_.getDomainOf(params(), params.depth);
    else if(req.action == "getRangeOf")
      set_res = onto_->class_graph_.getRangeOf(params(), params.depth);
    else if(req.action == "find")
      set2vector(onto_->class_graph_.find(params(), params.take_id), res.values);
    else if(req.action == "findSub")
      set2vector(onto_->class_graph_.findSub(params(), params.take_id), res.values);
    else if(req.action == "findRegex")
      set2vector(onto_->class_graph_.findRegex(params(), params.take_id), res.values);
    else if(req.action == "findFuzzy")
    {
      if(params.threshold != -1)
        set2vector(onto_->class_graph_.findFuzzy(params(), params.take_id, params.threshold), res.values);
      else
        set2vector(onto_->class_graph_.findFuzzy(params(), params.take_id), res.values);
    }
    else if(req.action == "exist")
    {
      if(onto_->class_graph_.touch(params()))
        res.values.push_back(params());
    }
    else
      res.code = UNKNOW_ACTION;

    if(params.selector != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") ||
        (req.action == "getDisjoint") || (req.action == "getOn") ||
        (req.action == "getFrom"))
        set_res = onto_->class_graph_.select(set_res, params.selector);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith") ||
              (req.action == "getDomainOf") || (req.action == "getRangeOf"))
        set_res = onto_->object_property_graph_.select(set_res, params.selector);
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
    removeUselessSpace(req.action);
    param_t params = getParams(req.param);

    //Remove for V2.4
    int level = getPropagationLevel(params.base);
    if(level != -1)
      params.depth = level;
    std::string select = getSelector(req.action, params.base);
    if(select != "")
      params.selector = select;
    // end remove for 2.4

    std::unordered_set<std::string> set_res;

    if(req.action == "getDown")
      set_res = onto_->object_property_graph_.getDown(params(), params.depth);
    else if(req.action == "getUp")
      set_res = onto_->object_property_graph_.getUp(params(), params.depth);
    else if(req.action == "getDisjoint")
      set_res = onto_->object_property_graph_.getDisjoint(params());
    else if(req.action == "getInverse")
      set_res = onto_->object_property_graph_.getInverse(params());
    else if(req.action == "getDomain")
      set_res = onto_->object_property_graph_.getDomain(params());
    else if(req.action == "getRange")
      set_res = onto_->object_property_graph_.getRange(params());
    else if(req.action == "getName")
    {
      auto tmp = onto_->object_property_graph_.getName(params(), params.take_id);
      if(tmp != "")
        res.values.push_back(tmp);
    }
    else if(req.action == "getNames")
      res.values = onto_->object_property_graph_.getNames(params());
    else if(req.action == "getEveryNames")
      res.values = onto_->object_property_graph_.getEveryNames(params());
    else if(req.action == "find")
      set2vector(onto_->object_property_graph_.find(params(), params.take_id), res.values);
    else if(req.action == "findSub")
      set2vector(onto_->object_property_graph_.findSub(params(), params.take_id), res.values);
    else if(req.action == "findRegex")
      set2vector(onto_->object_property_graph_.findRegex(params(), params.take_id), res.values);
    else if(req.action == "findFuzzy")
    {
      if(params.threshold != -1)
        set2vector(onto_->object_property_graph_.findFuzzy(params(), params.take_id, params.threshold), res.values);
      else
        set2vector(onto_->object_property_graph_.findFuzzy(params(), params.take_id), res.values);
    }
    else if(req.action == "exist")
    {
      if(onto_->object_property_graph_.touch(params()))
        res.values.push_back(params());
    }
    else
      res.code = UNKNOW_ACTION;

    if(params.selector != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") ||
        (req.action == "getDisjoint") || (req.action == "getInverse"))
        set_res = onto_->object_property_graph_.select(set_res, params.selector);
      else if((req.action == "getDomain") || (req.action == "getRange"))
        set_res = onto_->class_graph_.select(set_res, params.selector);
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
    removeUselessSpace(req.action);
    param_t params = getParams(req.param);

    //Remove for V2.4
    int level = getPropagationLevel(params.base);
    if(level != -1)
      params.depth = level;
    std::string select = getSelector(req.action, params.base);
    if(select != "")
      params.selector = select;
    // end remove for 2.4

    std::unordered_set<std::string> set_res;

    if(req.action == "getDown")
      set_res = onto_->data_property_graph_.getDown(params(), params.depth);
    else if(req.action == "getUp")
      set_res = onto_->data_property_graph_.getUp(params(), params.depth);
    else if(req.action == "getDisjoint")
      set_res = onto_->data_property_graph_.getDisjoint(params());
    else if(req.action == "getDomain")
      set_res = onto_->data_property_graph_.getDomain(params());
    else if(req.action == "getRange")
      set2vector(onto_->data_property_graph_.getRange(params()), res.values);
    else if(req.action == "getName")
    {
      auto tmp = onto_->data_property_graph_.getName(params(), params.take_id);
      if(tmp != "")
        res.values.push_back(tmp);
    }
    else if(req.action == "getNames")
      res.values = onto_->data_property_graph_.getNames(params());
    else if(req.action == "getEveryNames")
      res.values = onto_->data_property_graph_.getEveryNames(params());
    else if(req.action == "find")
      set2vector(onto_->data_property_graph_.find(params(), params.take_id), res.values);
    else if(req.action == "findSub")
      set2vector(onto_->data_property_graph_.findSub(params(), params.take_id), res.values);
    else if(req.action == "findRegex")
      set2vector(onto_->data_property_graph_.findRegex(params(), params.take_id), res.values);
    else if(req.action == "findFuzzy")
    {
      if(params.threshold != -1)
        set2vector(onto_->data_property_graph_.findFuzzy(params(), params.take_id, params.threshold), res.values);
      else
        set2vector(onto_->data_property_graph_.findFuzzy(params(), params.take_id), res.values);
    }
    else if(req.action == "exist")
    {
      if(onto_->data_property_graph_.touch(params()))
        res.values.push_back(params());
    }
    else
      res.code = UNKNOW_ACTION;

    if(params.selector != "")
    {
      if((req.action == "getUp") || (req.action == "getDown") || (req.action == "getDisjoint"))
        set_res = onto_->data_property_graph_.select(set_res, params.selector);
      else if(req.action == "getDomain")
        set_res = onto_->class_graph_.select(set_res, params.selector);
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
    removeUselessSpace(req.action);
    param_t params = getParams(req.param);

    //Remove for V2.4
    int level = getPropagationLevel(params.base);
    if(level != -1)
      params.depth = level;
    std::string select = getSelector(req.action, params.base);
    if(select != "")
      params.selector = select;
    // end remove for 2.4

    std::unordered_set<std::string> set_res;

    if(req.action == "getSame")
      set_res = onto_->individual_graph_.getSame(params());
    if(req.action == "getDistincts")
      set_res = onto_->individual_graph_.getDistincts(params());
    else if(req.action == "getRelationFrom")
      set_res = onto_->individual_graph_.getRelationFrom(params(), params.depth);
    else if(req.action == "getRelatedFrom")
      set_res = onto_->individual_graph_.getRelatedFrom(params());
    else if(req.action == "getRelationOn")
      set_res = onto_->individual_graph_.getRelationOn(params(), params.depth);
    else if(req.action == "getRelatedOn")
      set_res = onto_->individual_graph_.getRelatedOn(params());
    else if(req.action == "getRelationWith")
      set_res = onto_->individual_graph_.getRelationWith(params());
    else if(req.action == "getRelatedWith")
      set_res = onto_->individual_graph_.getRelatedWith(params());
    else if(req.action == "getUp")
      set_res = onto_->individual_graph_.getUp(params(), params.depth);
    else if(req.action == "getOn")
      set_res = onto_->individual_graph_.getOn(params());
    else if(req.action == "getFrom")
      set_res = onto_->individual_graph_.getFrom(params());
    else if(req.action == "getWith")
      set_res = onto_->individual_graph_.getWith(params(), params.depth);
    else if(req.action == "getDomainOf")
      set_res = onto_->individual_graph_.getDomainOf(params(), params.depth);
    else if(req.action == "getRangeOf")
      set_res = onto_->individual_graph_.getRangeOf(params(), params.depth);
    else if(req.action == "getName")
    {
      auto tmp = onto_->individual_graph_.getName(params(), params.take_id);
      if(tmp != "")
        res.values.push_back(tmp);
    }
    else if(req.action == "getNames")
      res.values = onto_->individual_graph_.getNames(params());
    else if(req.action == "getEveryNames")
      res.values = onto_->individual_graph_.getEveryNames(params());
    else if(req.action == "find")
      set_res = onto_->individual_graph_.find(params(), params.take_id);
    else if(req.action == "findSub")
      set_res = onto_->individual_graph_.findSub(params(), params.take_id);
    else if(req.action == "findRegex")
      set_res = onto_->individual_graph_.findRegex(params(), params.take_id);
    else if(req.action == "findFuzzy")
    {
      if(params.threshold != -1)
        set_res = onto_->individual_graph_.findFuzzy(params(), params.take_id, params.threshold);
      else
        set_res = onto_->individual_graph_.findFuzzy(params(), params.take_id);
    }
    else if(req.action == "getType")
      set_res = onto_->individual_graph_.getType(params());
    else if(req.action == "exist")
    {
      if(onto_->individual_graph_.touch(params()))
        res.values.push_back(params());
    }
    else
      res.code = UNKNOW_ACTION;

    if(params.selector != "")
    {
      if(req.action == "getUp")
        set_res = onto_->class_graph_.select(set_res, params.selector);
      else if((req.action == "getRelationFrom") || (req.action == "getRelationOn") || (req.action == "getWith") ||
              (req.action == "getDomainOf") || (req.action == "getRangeOf"))
        set_res = onto_->object_property_graph_.select(set_res, params.selector);
      else if((req.action != "find") || (req.action != "findRegex") || (req.action != "findSub") || (req.action != "findFuzzy") || (req.action != "getFrom") || (req.action != "getOn"))
        set_res = onto_->individual_graph_.select(set_res, params.selector);
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
  else if(req.action == "activeList")
    res.values = reasoners_.activeListVector();
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
  ros::Publisher feeder_publisher = n_->advertise<std_msgs::String>(publisher_name, PUB_QUEU_SIZE);

  ros::Rate wait(feeder_rate_);
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
        if(name_ != "")
          notif = "[" + name_ + "]" + notif;
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
  ros::Publisher reasoner_publisher = n_->advertise<std_msgs::String>(publisher_name, PUB_QUEU_SIZE);

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
    for(auto& notif : notifications)
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

bool RosInterface::split(const std::string &text, std::vector<std::string> &strs, const std::string& delim)
{
  std::string tmp_text = text;
  while(tmp_text.find(delim) != std::string::npos)
  {
    size_t pos = tmp_text.find(delim);
    std::string part = tmp_text.substr(0, pos);
    tmp_text = tmp_text.substr(pos + delim.size(), tmp_text.size() - pos - delim.size());
    if(part != "")
      strs.push_back(part);
  }
  strs.push_back(tmp_text);
  if(strs.size() > 1)
    return true;
  else
    return false;
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

param_t RosInterface::getParams(std::string& param)
{
  param_t parameters;
  std::vector<std::string> str_params;
  split(param, str_params, " ");

  if(str_params.size())
    parameters.base = str_params[0];

  bool option_found = false;
  for(size_t i = 1; i < str_params.size(); i++)
  {
    if((str_params[i] == "-d") || (str_params[i] == "--depth"))
    {
      i++;
      int tmp = -1;
      if(sscanf(str_params[i].c_str(), "%d", &tmp) != 1)
        tmp = -1;
      parameters.depth = tmp;
      option_found = true;
    }
    else if((str_params[i] == "-s") || (str_params[i] == "--selector"))
    {
      i++;
      parameters.selector = str_params[i];
      option_found = true;
    }
    else if((str_params[i] == "-t") || (str_params[i] == "--threshold"))
    {
      i++;
      float tmp = -1;
      if(sscanf(str_params[i].c_str(), "%f", &tmp) != 1)
        tmp = -1;
      parameters.threshold = tmp;
      option_found = true;
    }
    else if((str_params[i] == "-i") || (str_params[i] == "--take_id"))
    {
      i++;
      bool tmp = false;
      if(str_params[i] == "true")
        tmp = true;

      parameters.take_id = tmp;
      option_found = true;
    }
    else if(option_found)
      std::cout << "[WARNING] unknow parameter \"" << str_params[i] << "\"" << std::endl;
    else
      parameters.base += " " + str_params[i];
  }

  return parameters;
}

int RosInterface::getPropagationLevel(std::string& params)
{
  size_t delimitater = params.find('<');
  if(delimitater != std::string::npos)
  {
    std::cout << "[WARNING] Deprecated propagation level definition. Use -d option" << std::endl;
    std::string param = params.substr(0, delimitater);
    std::string level = params.substr(delimitater+1);
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
    std::cout << "[WARNING] Deprecated selector definition. Use -s option" << std::endl;
    action = action.substr(std::string("select:").size());
    size_t delimitater = param.find('=');
    if(delimitater != std::string::npos)
    {
      select = param.substr(0, delimitater);
      param = param.substr(delimitater+1);

    }
    removeUselessSpace(action);
  }
  return select;
}

} // namespace ontologenius
