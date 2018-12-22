#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <string>
#include <vector>
#include <unordered_set>
#include <atomic>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ontologenius/OntologeniusService.h"

#include "ontoloGenius/core/ontoGraphs/Ontology.h"
#include "ontoloGenius/core/reasoner/Reasoners.h"
#include "ontoloGenius/core/feeder/Feeder.h"

class RosInterface
{
public:
  RosInterface(ros::NodeHandle* n, std::string name = "");
  ~RosInterface();

  void init(std::string& lang, std::string intern_file, std::vector<std::string>& files);
  void run();
  void stop() {run_ = false; }
  inline bool isRunning() {return run_; }
  Ontology* getOntology() {return onto_; }

private:
  ros::NodeHandle* n_;
  Ontology* onto_;
  Reasoners reasoners_;
  Feeder feeder_;

  std::string name_;
  std::atomic<bool> run_;

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
  void set2string(const std::unordered_set<std::string>& word_set, std::string& result);
  void set2vector(const std::unordered_set<std::string>& word_set, std::vector<std::string>& result);
  int getPropagationLevel(std::string& params);
  std::string getSelector(std::string& action, std::string& param);
};

#endif
