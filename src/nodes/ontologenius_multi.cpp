#include "ros/ros.h"
#include <thread>
#include <regex>

#include "ontoloGenius/RosInterface.h"
#include "ontoloGenius/core/ontologyOperators/differenceFinder.h"
#include "ontoloGenius/core/utility/error_code.h"

void removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.size() != 0))
    text.erase(0,1);

  while((text[text.size() - 1] == ' ') && (text.size() != 0))
    text.erase(text.size() - 1,1);
}

ros::NodeHandle* n_;
std::map<std::string, RosInterface*> interfaces_;
std::map<std::string, std::thread> interfaces_threads_;

std::string language;
std::string intern_file = "none";
std::vector<std::string> files;

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

  differenceFinder diff;
  std::regex base_regex("(.*)\\|(.*)\\|(.*)");
  std::smatch base_match;
  if (std::regex_match(param, base_match, base_regex))
  {
    if (base_match.size() == 4)
    {
      std::string onto1 = base_match[1].str();
      Ontology* onto1_ptr;
      auto it = interfaces_.find(onto1);
      if(it == interfaces_.end())
      {
        res_code = NO_EFFECT;
        return res;
      }
      else
        onto1_ptr = interfaces_[onto1]->getOntology();

      std::string onto2 = base_match[2].str();
      Ontology* onto2_ptr;
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
      RosInterface* tmp = new RosInterface(n_, req.param);
      interfaces_[req.param] = tmp;
      tmp->init(language, intern_file, files);
      std::thread th(&RosInterface::run, tmp);
      interfaces_threads_[req.param] = std::move(th);

      std::cout << req.param << " STARTED" << std::endl;
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

  ros::service::waitForService("ontologenius/rest", -1);

  language = std::string(argv[1]);
  std::cout << "language " << language << std::endl;

  intern_file = std::string(argv[2]);
  std::cout << "intern_file " << intern_file << std::endl;

  for(int i = 3; i < argc; i++)
    files.push_back(std::string(argv[i]));

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
