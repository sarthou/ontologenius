#ifndef ONTOLOGENIUS_ROSINTERFACE_H
#define ONTOLOGENIUS_ROSINTERFACE_H

#include <string>
#include <vector>
#include <unordered_set>
#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "ontologenius/OntologeniusService.h"

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/reasoner/Reasoners.h"
#include "ontologenius/core/feeder/Feeder.h"

namespace ontologenius {

struct param_t
{
  std::string base;
  size_t depth;
  std::string selector;
  double threshold;
  bool take_id;

  param_t(): depth(-1), threshold(-1), take_id(true) {}

  std::string operator()() { return base; }
};

class RosInterface
{
public:
  RosInterface(ros::NodeHandle* n, std::string name = "");
  RosInterface(RosInterface& other, ros::NodeHandle* n, std::string name = "");
  ~RosInterface();

  void init(const std::string& lang, const std::string& intern_file, const std::vector<std::string>& files, const std::string& config_path);
  void init(const std::string& lang, const std::string& config_path);
  void run();
  void stop() {run_ = false; }
  inline bool isRunning() {return run_; }
  Ontology* getOntology() {return onto_; }

  void lock();
  void release();

private:
  ros::NodeHandle* n_;
  Ontology* onto_;
  Reasoners reasoners_;
  Feeder feeder_;

  std::string name_;
  std::atomic<bool> run_;
  bool feeder_end;
  size_t feeder_rate_;
  ros::Publisher pub_;

  std::mutex feeder_mutex_;
  std::mutex reasoner_mutex_;

  void knowledgeCallback(const std_msgs::String::ConstPtr& msg);

  bool actionsHandle(ontologenius::OntologeniusService::Request &req,
                     ontologenius::OntologeniusService::Response &res);
  bool classHandle(ontologenius::OntologeniusService::Request &req,
                   ontologenius::OntologeniusService::Response &res);
  bool objectPropertyHandle(ontologenius::OntologeniusService::Request &req,
                            ontologenius::OntologeniusService::Response &res);
  bool dataPropertyHandle(ontologenius::OntologeniusService::Request &req,
                          ontologenius::OntologeniusService::Response &res);
  bool individualHandle(ontologenius::OntologeniusService::Request  &req,
                        ontologenius::OntologeniusService::Response &res);
  bool reasonerHandle(ontologenius::OntologeniusService::Request &req,
                    ontologenius::OntologeniusService::Response &res);

  void feedThread();
  void periodicReasoning();

  void removeUselessSpace(std::string& text);
  bool split(const std::string &text, std::vector<std::string> &strs, const std::string& delim);
  void set2string(const std::unordered_set<std::string>& word_set, std::string& result);
  void set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result);
  param_t getParams(std::string& param);
  int getPropagationLevel(std::string& params);
  std::string getSelector(std::string& action, std::string& param);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ROSINTERFACE_H
