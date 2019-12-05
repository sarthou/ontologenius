#include <thread>
#include <regex>

#include <ros/ros.h>

#include "ontologenius/RosInterface.h"
#include "ontologenius/core/ontologyOperators/differenceFinder.h"
#include "ontologenius/Parameters.h"
#include "ontologenius/core/utility/error_code.h"

void removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.size() != 0))
    text.erase(0,1);

  while((text[text.size() - 1] == ' ') && (text.size() != 0))
    text.erase(text.size() - 1,1);
}

ros::NodeHandle* n_;
std::map<std::string, ontologenius::RosInterface*> interfaces_;
std::map<std::string, std::thread> interfaces_threads_;

ontologenius::Parameters params;

bool deleteInterface(std::string name)
{
  interfaces_[name]->stop();
  try
  {
    interfaces_threads_[name].join();
  }
  catch(...)
  {
    return false;
  }

  interfaces_threads_.erase(name);
  delete interfaces_[name];
  interfaces_.erase(name);

  std::cout << name << " STOPED" << std::endl;
  return true;
}

std::vector<std::string> getDiff(std::string& param, int& res_code)
{
  std::vector<std::string> res;

  ontologenius::differenceFinder diff;
  std::regex base_regex("(.*)\\|(.*)\\|(.*)");
  std::smatch base_match;
  if (std::regex_match(param, base_match, base_regex))
  {
    if (base_match.size() == 4)
    {
      std::string onto1 = base_match[1].str();
      ontologenius::Ontology* onto1_ptr;
      auto it = interfaces_.find(onto1);
      if(it == interfaces_.end())
      {
        res_code = NO_EFFECT;
        return res;
      }
      else
        onto1_ptr = interfaces_[onto1]->getOntology();

      std::string onto2 = base_match[2].str();
      ontologenius::Ontology* onto2_ptr;
      it = interfaces_.find(onto2);
      if(it == interfaces_.end())
      {
        res_code = NO_EFFECT;
        return res;
      }
      else
        onto2_ptr = interfaces_[onto2]->getOntology();

      std::string concept = base_match[3].str();
      res = diff.getDiff(onto1_ptr, onto2_ptr, concept);
    }
  }
  else
    res_code = UNKNOW_ACTION;

  return res;
}

bool managerHandle(ontologenius::OntologeniusService::Request &req,
                   ontologenius::OntologeniusService::Response &res)
{
  res.code = 0;

  removeUselessSpace(req.action);
  removeUselessSpace(req.param);

  if(req.action == "add")
  {
    auto it = interfaces_.find(req.param);
    if(it != interfaces_.end())
      res.code = NO_EFFECT;
    else
    {
      ontologenius::RosInterface* tmp = new ontologenius::RosInterface(n_, req.param);
      interfaces_[req.param] = tmp;
      tmp->init(params.parameters_.at("language").getFirst(),
                params.parameters_.at("intern_file").getFirst(),
                params.parameters_.at("files").get(),
                params.parameters_.at("config").getFirst());
      std::thread th(&ontologenius::RosInterface::run, tmp);
      interfaces_threads_[req.param] = std::move(th);

      std::cout << req.param << " STARTED" << std::endl;
    }
  }
  else if(req.action == "copy")
  {
    std::regex base_regex("(.*)=(.*)");
    std::smatch base_match;
    if (std::regex_match(req.param, base_match, base_regex))
    {
      if (base_match.size() == 3)
      {
        std::string base_name = base_match[2].str();
        std::string copy_name = base_match[1].str();
        auto it = interfaces_.find(base_name);
        if(it == interfaces_.end()) // if interface to copy do not exist
          res.code = NO_EFFECT;
        else
        {
          it = interfaces_.find(copy_name);
          if(it != interfaces_.end()) // if copy already exist
            res.code = NO_EFFECT;
          else
          {
            ontologenius::RosInterface* tmp = new ontologenius::RosInterface(*(interfaces_[base_name]), n_, copy_name);
            interfaces_[copy_name] = tmp;
            tmp->init(params.parameters_.at("language").getFirst(),
                      params.parameters_.at("config").getFirst());
            std::thread th(&ontologenius::RosInterface::run, tmp);
            interfaces_threads_[copy_name] = std::move(th);

            std::cout << copy_name << " STARTED" << std::endl;
          }
        }
      }
    }
  }
  else if(req.action == "delete")
  {
    auto it = interfaces_.find(req.param);
    if(it == interfaces_.end())
      res.code = NO_EFFECT;
    else
    {
      if(deleteInterface(req.param) == false)
        res.code = REQUEST_ERROR;
    }
  }
  else if(req.action == "list")
  {
    for(auto it : interfaces_)
      res.values.push_back(it.first);
  }
  else if(req.action == "difference")
  {
    int code = 0;
    res.values = getDiff(req.param, code);
    res.code = code;
  }
  else
    res.code = UNKNOW_ACTION;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_multi");

  ros::NodeHandle n;
  n_ = &n;

  params.insert(ontologenius::Parameter("language", {"-l", "--lang"}, {"en"}));
  params.insert(ontologenius::Parameter("intern_file", {"-i", "--intern_file"}, {"none"}));
  params.insert(ontologenius::Parameter("config", {"-c", "--config"}, {"none"}));
  params.insert(ontologenius::Parameter("files", {}));

  params.set(argc, argv);
  params.display();

  ros::service::waitForService("ontologenius/rest", -1);

  ros::ServiceServer service = n_->advertiseService("ontologenius/manage", managerHandle);

  ros::spin();

  std::vector<std::string> interfaces_names;
  for(auto intreface : interfaces_)
    interfaces_names.push_back(intreface.first);

  for(size_t i = 0; i < interfaces_names.size(); i++)
    deleteInterface(interfaces_names[i]);

  ROS_DEBUG("KILL ontoloGenius");

  return 0;
}
