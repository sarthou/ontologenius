#ifndef ONTOLOGENIUS_ROSINTERFACE_H
#define ONTOLOGENIUS_ROSINTERFACE_H

#include <string>
#include <vector>
#include <unordered_set>
#include <atomic>
#include <mutex>
#include <thread>

#include "ontologenius/compat/ros.h"

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/reasoner/Reasoners.h"
#include "ontologenius/core/feeder/Feeder.h"
#include "ontologenius/core/feeder/FeederEcho.h"
#include "ontologenius/core/ontologyOperators/Sparql.h"

//#define ONTO_TEST

namespace ontologenius {

class RosInterface
{
public:
  /// @brief The RosInterface constructor
  /// @param name is the name of the created ontology instance. Two instances can not have the same name
  RosInterface(const std::string& name = "");
  /// @brief The RosInterface copy constructor
  /// @param other is the interface to copy
  /// @param name is the name of the created ontology instance. Two instances can not have the same name
  RosInterface(RosInterface& other, const std::string& name = "");
  /// @brief The RosInterface destructor
  ~RosInterface();

  RosInterface& operator=(const RosInterface& other) = delete;

  /// @brief Initializes the interface
  /// @param lang is the used language for the natural language names of the concepts
  /// @param intern_file is the path to the file in which the ontology will be stored at the end of the process. Could be set to "none" to not use it
  /// @param files is a vector of paths to the files to load if no intern file exist
  /// @param config_path is the path to the yaml file used to configure the reasoners. Could be set to "none" if you do not use any configuration file
  void init(const std::string& lang, const std::string& intern_file, const std::vector<std::string>& files, const std::string& config_path);
  /// @brief Initializes the interface created by a copy of an existing ontology
  /// @param lang is the used language for the natural language names of the concepts
  /// @param config_path is the path to the yaml file used to configure the reasoners. Could be set to "none" if you do not use any configuration file
  void init(const std::string& lang, const std::string& config_path);
  /// @brief Starts the interface. This function is blocking. You will thus stay in until you stop the process with the stop function or kill the process
  void run();
  /// @brief Stops the interface.
  void stop();

  /// @brief Closes the ontology to make it explorable. This is not mandatory to close it in this way. It is better to let the user do it by himself
  bool close();

  /// @brief Tests if the interface is running. By running we meant that the threads and ROS subscribers are alive
  /// @return true is the interface is running, false otherwise
  inline bool isRunning() { return run_; }
  /// @brief Gets a pointer on the used ontology to use it externally. Using it externally can be dangerous, be careful
  /// @return A pointer on an ontology object
  Ontology* getOntology() { return onto_; }
  /// @brief Gets a pointer on the SPARQL interface.
  /// @return A pointer on the SPARQL interface object
  Sparql* getSparqlInterface() { return &sparql_; }

  /// @brief Lock the mutex related to the feeder thread and the reasoner threads. Must be used if you use the ontology externally
  void lock();
  /// @brief Unlock the mutex related to the feeder thread and the reasoner threads. Must be used if you use the ontology externally
  void release();

  /// @brief Allows or not debug display
  /// @param display should be set to false to not allow debug display
  void setDisplay(bool display);

#ifndef ONTO_TEST
private:
#else
public:
#endif
  /// @brief The graph representing the ontology
  Ontology* onto_;
  /// @brief The reasoners pool. Reasoners are automatically load as plugins
  Reasoners reasoners_;
  /// @brief Analyses incoming statements and manage the versioning system
  Feeder feeder_;
  /// @brief Republishs the incoming statement and the deduced once
  FeederEcho feeder_echo_;
  /// @brief Resolves SPARQL queries.
  Sparql sparql_;

#ifdef ONTO_TEST
  bool end_feed_;
#endif

  /// @brief The ontology instance name
  std::string name_;
  /// @brief This varible is set to true when the interface is started and should be set to false to stop it
  std::atomic<bool> run_;
  /// @brief The rate (in Hz) at which the feeder thread run. Can not be modified while the process is running
  size_t feeder_rate_;
  
  compat::onto_ros::Publisher<std_msgs_compat::String> feeder_end_pub_;

  /// @brief The mutex protecting the object feeder_
  std::mutex feeder_mutex_;
  /// @brief The mutex protecting the object reasoners_
  std::mutex reasoner_mutex_;

  /// @brief The variable used to display or not debug information. Can be changed at run time
  bool display_;
  /// @brief represents all files provided at the initialization
  std::vector<std::string> files_;
  /// @brief represents the dedicated intern file
  std::string intern_file_;

  /// @brief The ROS topic callback receiving statements not stamped
  void knowledgeCallback(compat::onto_ros::MessageWrapper<std_msgs_compat::String> msg);
  /// @brief The ROS topic callback receiving stamped statements
  void stampedKnowledgeCallback(compat::onto_ros::MessageWrapper<compat::OntologeniusStampedString> msg);

  /// @brief The ROS service callback in charge of general operations on the ontology
  bool actionsHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request>& req,
                     compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response>& res);

  /// @brief The ROS service callback in charge of the exploration on classes
  bool classHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request> &req,
                   compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response> &res);
  /// @brief The ROS service callback in charge of the exploration on object properties
  bool objectPropertyHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request> &req,
                            compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response> &res);
  /// @brief The ROS service callback in charge of the exploration on data properties
  bool dataPropertyHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request> &req,
                          compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response> &res);
  /// @brief The ROS service callback in charge of the exploration on individuals
  bool individualHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request>  &req,
                        compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response> &res);
  /// @brief The ROS service callback in charge of the SPARQL queries
  bool sparqlHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlService::Request> &req,
                    compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlService::Response> &res);

  /// @brief The ROS service callback in charge of the exploration on classes with indexes
  bool classIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request> &req,
                        compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response> &res);
  /// @brief The ROS service callback in charge of the exploration on object properties with indexes
  bool objectPropertyIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request> &req,
                                 compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response> &res);
  /// @brief The ROS service callback in charge of the exploration on data properties with indexes
  bool dataPropertyIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request> &req,
                               compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response> &res);
  /// @brief The ROS service callback in charge of the exploration on individuals with indexes
  bool individualIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>  &req,
                             compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response> &res);
  /// @brief The ROS service callback in charge of the SPARQL queries with indexes
  bool sparqlIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlIndexService::Request>& req,
                         compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlIndexService::Response>& res);

  /// @brief The ROS service callback in charge of the reasoners
  bool reasonerHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request> &req,
                      compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response> &res);

  /// @brief The ROS service callback in charge of the conversion from index to identifier and inverse
  bool conversionHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusConversion::Request> &req,
                        compat::onto_ros::ServiceWrapper<compat::OntologeniusConversion::Response> &res);

  /// @brief The thread that periodically manages the update of the ontology with the incoming instructions 
  void feedThread();
  /// @brief The thread that run the periodic reasoners
  void periodicReasoning();

  /// @brief Removes usless spaces at the begin and end of a string
  /// @param text is the string on which you want to remove the spaces
  void removeUselessSpace(std::string& text);
  /// @brief Convert a set of strings into a string in which each element is separated by the symbol '-'
  /// @param word_set is the set to convert
  /// @param result is the string to which to add the elements of the set 
  void set2string(const std::unordered_set<std::string>& word_set, std::string& result);
  /// @brief Convert a set of T into a vector of T
  /// @param word_set is the set to convert
  /// @param result is the vector of T to which to add the elements of the set
  template <typename T>
  void set2vector(const std::unordered_set<T>& word_set, std::vector<T>& result)
  {
    std::copy(word_set.begin(), word_set.end(), std::back_inserter(result));
  }

  /// @brief Gets a topic name related corresponding to the current instance name
  /// @param topic_name is the name of the topic you want to create
  std::string getTopicName(const std::string& topic_name)
  {
    return getTopicName(topic_name, name_);
  }

  /// @brief Gets a topic name related corresponding to a given instance name
  /// @param topic_name is the name of the topic you want to create
  /// @param onto_name is the name of the ontology to which the topic should be related to
  std::string getTopicName(const std::string& topic_name, const std::string& onto_name)
  {
    return (onto_name == "") ? "ontologenius/" + topic_name : "ontologenius/" + topic_name + "/" + onto_name;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ROSINTERFACE_H
