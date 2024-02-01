#include "ontologenius/interface/RosInterface.h"

#include "ontologenius/utils/String.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"

#define PUB_QUEU_SIZE 1000
#define SUB_QUEU_SIZE 10000

#define FEEDER_DEFAULT_RATE 20
#define FEEDER_COPY_RATE 4000

namespace ontologenius {

RosInterface::RosInterface(const std::string& name) :
        reasoners_(name),
        feeder_echo_(getTopicName("insert_echo", name), getTopicName("insert_explanations", name)),
#ifdef ONTO_TEST
        end_feed_(true),
#endif
        run_(true),
        feeder_rate_(FEEDER_DEFAULT_RATE),
        feeder_end_pub_(getTopicName("end", name), PUB_QUEU_SIZE),
        display_(true)
{
  onto_ = new Ontology();
  onto_->setDisplay(display_);
  reasoners_.link(onto_);
  feeder_.link(onto_);
  sparql_.link(onto_);

  name_ = name;
  // n_.setCallbackQueue(&callback_queue_);
}

RosInterface::RosInterface(RosInterface& other, const std::string& name) :
        reasoners_(name),
        feeder_echo_(getTopicName("insert_echo", name), getTopicName("insert_explanations", name)),
#ifdef ONTO_TEST
        end_feed_(true),
#endif
        run_(true),
        feeder_rate_(FEEDER_DEFAULT_RATE),
        feeder_end_pub_(getTopicName("end", name), PUB_QUEU_SIZE),
        display_(true)
{
  other.lock();
  onto_ = new Ontology(*other.onto_);
  onto_->setDisplay(display_);
  other.release();

  reasoners_.link(onto_);
  feeder_.link(onto_);
  feeder_.setVersioning(true);
  sparql_.link(onto_);

  name_ = name;
  // n_.setCallbackQueue(&callback_queue_);
}

RosInterface::~RosInterface()
{
  lock();
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
  intern_file_ = dedicated_intern_file;

  files_ = files;
  if(onto_->preload(dedicated_intern_file) == false)
    for(auto& file : files)
      onto_->readFromFile(file);

  reasoners_.configure(config_path);
  reasoners_.load();
  Display::info("Plugins loaded : " + reasoners_.list());

  feeder_rate_ = FEEDER_DEFAULT_RATE;
}

void RosInterface::init(const std::string& lang, const std::string& config_path)
{
  onto_->setLanguage(lang);

  reasoners_.configure(config_path);
  reasoners_.load();
  Display::info("Plugins loaded : " + reasoners_.list());

  feeder_.activateVersionning(true);
  feeder_rate_ = FEEDER_COPY_RATE;
}

void RosInterface::run()
{
  auto sub_insert = compat::onto_ros::Subscriber<std_msgs_compat::String>(
    getTopicName("insert"),
    PUB_QUEU_SIZE,
    &RosInterface::knowledgeCallback,
    this
  );
  (void) sub_insert;

  auto sub_insert_stamped = compat::onto_ros::Subscriber<compat::OntologeniusStampedString>(
    getTopicName("insert_stamped"),
    PUB_QUEU_SIZE,
    &RosInterface::stampedKnowledgeCallback,
    this
  );
  (void) sub_insert_stamped;

  std::vector<compat::onto_ros::Service<compat::OntologeniusService>> str_services;
  str_services.emplace_back(getTopicName("actions"),               &RosInterface::actionsHandle, this);
  str_services.emplace_back(getTopicName("reasoner"),              &RosInterface::reasonerHandle, this);

  str_services.emplace_back(getTopicName("class"),                 &RosInterface::classHandle, this);
  str_services.emplace_back(getTopicName("object_property"),       &RosInterface::objectPropertyHandle, this);
  str_services.emplace_back(getTopicName("data_property"),         &RosInterface::dataPropertyHandle, this);
  str_services.emplace_back(getTopicName("individual"),            &RosInterface::individualHandle, this);

  std::vector<compat::onto_ros::Service<compat::OntologeniusIndexService>> idx_services;
  idx_services.emplace_back(getTopicName("class_index"),           &RosInterface::classIndexHandle, this);
  idx_services.emplace_back(getTopicName("object_property_index"), &RosInterface::objectPropertyIndexHandle, this);
  idx_services.emplace_back(getTopicName("data_property_index"),   &RosInterface::dataPropertyIndexHandle, this);
  idx_services.emplace_back(getTopicName("individual_index"),      &RosInterface::individualIndexHandle, this);

  compat::onto_ros::Service<compat::OntologeniusConversion> srv_conversion(
    getTopicName("conversion"), &RosInterface::conversionHandle, this);
  (void) srv_conversion;

  auto srv_sparql = compat::onto_ros::Service<compat::OntologeniusSparqlService>(
    getTopicName("sparql"), &RosInterface::sparqlHandle, this
  );
  (void) srv_sparql;

  auto srv_sparql_idx = compat::onto_ros::Service<compat::OntologeniusSparqlIndexService>(
    getTopicName("sparql_index"), &RosInterface::sparqlIndexHandle, this
  );
  (void) srv_sparql_idx;

  std::thread feed_thread(&RosInterface::feedThread, this);
  std::thread periodic_reasoning_thread(&RosInterface::periodicReasoning, this);

  if(name_ != "")
    Display::info(name_ + " is ready");
  else
    Display::info("Ontologenius is ready");

  periodic_reasoning_thread.join();
  feed_thread.join();
}

void RosInterface::stop()
{
  // node_handle->now();
  run_ = false;
  /*callback_queue_.disable();
  callback_queue_.clear();*/
}

void RosInterface::lock()
{
  feeder_mutex_.lock();
  reasoner_mutex_.lock();
}

void RosInterface::release()
{
  reasoner_mutex_.unlock();
  feeder_mutex_.unlock();
}

bool RosInterface::close()
{
  if(onto_->close() == false)
    return false;
  reasoners_.initialize();
  reasoners_.runPostReasoners();

  reasoner_mutex_.lock();
  reasoners_.getExplanations(); // flush explanations of initialization
  reasoner_mutex_.unlock();

  return true;
}

void RosInterface::setDisplay(bool display)
{
  display_ = display;
  onto_->setDisplay(display_);
}

/***************
*
* Callbacks
*
****************/

void RosInterface::knowledgeCallback(compat::onto_ros::MessageWrapper<std_msgs_compat::String> msg)
{
  auto now = compat::onto_ros::Node::get().current_time();
  
  feeder_.store(msg->data, { (uint32_t) now.seconds(), (uint32_t) now.nanoseconds()});
}

void RosInterface::stampedKnowledgeCallback(compat::onto_ros::MessageWrapper<compat::OntologeniusStampedString> msg)
{
  feeder_.store(msg->data, {msg->stamp.seconds, msg->stamp.nanoseconds});
}

bool RosInterface::actionsHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request>& req,
                                  compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response>& res)
{
  return [this](auto&& req, auto&& res)
  {
    res->code = 0;

    removeUselessSpace(req->action);
    removeUselessSpace(req->param);

    if(req->action == "add")
      res->code = onto_->readFromUri(req->param);
    else if(req->action == "fadd")
      res->code = onto_->readFromFile(req->param);
    else if(req->action == "save")
      onto_->save(req->param);
    else if(req->action == "export")
      feeder_.exportToXml(req->param);
    else if(req->action == "close")
    {
      if(close() == false)
        res->code = REQUEST_ERROR;
    }
    else if(req->action == "reset")
    {
      lock();
      delete onto_;
      onto_ = new Ontology();
      onto_->setDisplay(display_);
      reasoners_.link(onto_);
      feeder_.link(onto_);
      sparql_.link(onto_);
      
      if(onto_->preload(intern_file_) == false)
        for(auto& file : files_)
          onto_->readFromFile(file);
      release();
    }
    else if(req->action == "clear")
    {
      lock();
      delete onto_;
      onto_ = new Ontology();
      onto_->setDisplay(display_);
      reasoners_.link(onto_);
      feeder_.link(onto_);
      sparql_.link(onto_);
      release();
    }
    else if(req->action == "setLang")
      onto_->setLanguage(req->param);
    else if(req->action == "getLang")
      res->values.push_back(onto_->getLanguage());
    else
      res->code = UNKNOW_ACTION;

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

bool RosInterface::reasonerHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request> &req,
                                  compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response> &res)
{
  return [this](auto&& req, auto&& res)
  {
    res->code = 0;

    reasoner_mutex_.lock();
    if(req->action == "activate")
      res->code = reasoners_.activate(req->param);
    else if(req->action == "deactivate")
      res->code = reasoners_.deactivate(req->param);
    else if(req->action == "list")
      res->values = reasoners_.listVector();
    else if(req->action == "activeList")
      res->values = reasoners_.activeListVector();
    else if(req->action == "getDescription")
      res->values.push_back(reasoners_.getDescription(req->param));
    else
      res->code = UNKNOW_ACTION;
    reasoner_mutex_.unlock();

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

bool RosInterface::conversionHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusConversion::Request> &req,
                                    compat::onto_ros::ServiceWrapper<compat::OntologeniusConversion::Response> &res)
{
  return [this](auto&& req, auto&& res)
  {
    if(req->values_int.size())
    {
      if(req->source == req->INDIVIDUALS)
        res->values_str = onto_->individual_graph_.getIdentifiers(req->values_int);
      else if(req->source == req->CLASSES)
          res->values_str = onto_->class_graph_.getIdentifiers(req->values_int);
      else if(req->source == req->DATA_PROPERTIES)
        res->values_str = onto_->data_property_graph_.getIdentifiers(req->values_int);
      else if(req->source == req->OBJECT_PROPERTIES)
        res->values_str = onto_->object_property_graph_.getIdentifiers(req->values_int);
      else if(req->source == req->LITERAL)
        res->values_str = onto_->data_property_graph_.getLiteralIdentifiers(req->values_int);
      else
        return false;
    }

    if(req->values_str.size())
    {
      if(req->source == req->INDIVIDUALS)
        res->values_int = onto_->individual_graph_.getIndexes(req->values_str);
      else if(req->source == req->CLASSES)
        res->values_int = onto_->class_graph_.getIndexes(req->values_str);
      else if(req->source == req->DATA_PROPERTIES)
        res->values_int = onto_->data_property_graph_.getIndexes(req->values_str);
      else if(req->source == req->OBJECT_PROPERTIES)
        res->values_int = onto_->object_property_graph_.getIndexes(req->values_str);
      else if(req->source == req->LITERAL)
        res->values_int = onto_->data_property_graph_.getLiteralIndexes(req->values_str);
      else
        return false;
    }

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

/***************
*
* Threads
*
***************/

void RosInterface::feedThread()
{
  auto pub_feeder = compat::onto_ros::Publisher<std_msgs_compat::String>(getTopicName("feeder_notifications"), PUB_QUEU_SIZE);

  bool feeder_end = true;
#ifdef ONTO_TEST
  end_feed_ = false;
#endif

  compat::onto_ros::Rate wait(feeder_rate_);
  while(compat::onto_ros::Node::ok() && (onto_->isInit(false) == false) && (run_ == true))
  {
    wait.sleep();
  }

  auto& node = compat::onto_ros::Node::get();

  auto time = node.current_time();
  feeder_.store("[add]myself|", {(uint32_t) time.seconds(), (uint32_t) time.nanoseconds()});
  if(name_ != "")
    feeder_.store("[add]myself|=|" + name_, {(uint32_t)time.seconds(), (uint32_t)time.nanoseconds()});
  feeder_mutex_.lock();
  feeder_.run();
  feeder_mutex_.unlock();

  std_msgs_compat::String msg;

  while (compat::onto_ros::Node::ok() && (run_ == true))
  {
    feeder_mutex_.lock();
    bool run = feeder_.run();
    if((run == true) && (run_ == true))
    {
      if(feeder_end == true)
        feeder_end = false;

      std::vector<std::string> notifications = feeder_.getNotifications();
      for(auto notif : notifications)
      {
        Display::error("[Feeder]" + notif);
        if(name_ != "")
          notif = "[" + name_ + "]" + notif;
        msg.data = notif;
        pub_feeder.publish(msg);
      }

      auto echo = feeder_.getValidRelations();
      feeder_echo_.add(echo);
      auto explanations = feeder_.getExplanations();
      feeder_echo_.add(explanations);
    }
    else if(feeder_end == false)
    {
      reasoner_mutex_.lock();
      reasoners_.runPostReasoners();
      reasoner_mutex_.unlock();
#ifdef ONTO_TEST
      end_feed_ = true;
#endif

      feeder_end = true;
      msg.data = "end";
      feeder_end_pub_.publish(msg);
    }

    if((run == true) && (run_ == true))
      feeder_echo_.publish();

    feeder_mutex_.unlock();

    if (compat::onto_ros::Node::ok() && (run_ == true) && (run == false))
      wait.sleep();
  }
}

void RosInterface::periodicReasoning()
{
  auto pub_reasoner = compat::onto_ros::Publisher<std_msgs_compat::String>(getTopicName("reasoner_notifications", name_), PUB_QUEU_SIZE);
  auto pub_explanation = compat::onto_ros::Publisher<compat::OntologeniusExplanation>(getTopicName("insert_explanations", name_), PUB_QUEU_SIZE);

  compat::onto_ros::Rate wait(10);

  while (compat::onto_ros::Node::ok() && (onto_->isInit(false) == false) && (run_ == true))
  {
      wait.sleep();
  }

  compat::onto_ros::Rate r(100);
  std_msgs_compat::String msg;

  compat::OntologeniusExplanation expl_msg;

  while (compat::onto_ros::Node::ok() && (run_ == true))
  {
    reasoner_mutex_.lock();
    reasoners_.runPeriodicReasoners();

    auto notifications = reasoners_.getNotifications();
    for(auto& notif : notifications)
    {
      switch(notif.first)
      {
        case notification_debug: std::cout << "[Reasoners]" << notif.second << std::endl; break;
        case notification_info: Display::info("[Reasoners]" + notif.second); break;
        case notification_warning: Display::warning("[Reasoners]" + notif.second); break;
        case notification_error: Display::error("[Reasoners]" + notif.second); break;
      }
      msg.data = notif.second;
      pub_reasoner.publish(msg);
    }

    auto explanations = reasoners_.getExplanations();
    for(auto& explanation : explanations)
    {
      expl_msg.fact = explanation.first;
      expl_msg.cause = explanation.second;
      pub_explanation.publish(expl_msg);
    }

    reasoner_mutex_.unlock();

    if (compat::onto_ros::Node::ok() && (run_ == true)) r.sleep();
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

} // namespace ontologenius