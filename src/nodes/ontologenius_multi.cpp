#include <algorithm>
#include <array>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/ontologyOperators/DifferenceFinder.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"
#include "ontologenius/interface/RosInterface.h"
#include "ontologenius/utils/Parameters.h"

void handler(int sig)
{
  std::array<void*, 10> array;
  const int size = backtrace(array.data(), 10);

  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array.data(), size, STDERR_FILENO);
  exit(1);
}

void removeUselessSpace(std::string& text)
{
  while((text[0] == ' ') && (text.empty() == false))
    text.erase(0, 1);

  while((text[text.size() - 1] == ' ') && (text.empty() == false))
    text.erase(text.size() - 1, 1);
}

std::map<std::string, ontologenius::RosInterface*> interfaces;
std::map<std::string, std::thread> interfaces_threads;

ontologenius::Parameters params;

bool deleteInterface(const std::string& name)
{
  interfaces[name]->stop();
  usleep(200000);
  try
  {
    interfaces_threads[name].join();
  }
  catch(std::runtime_error& ex)
  {
    ontologenius::Display::error("An error was caught while joining the interface thread : " + std::string(ex.what()));
    ontologenius::Display::warning("The thread will be detached");
    interfaces_threads[name].detach();
  }

  interfaces_threads.erase(name);
  delete interfaces[name];
  interfaces.erase(name);

  std::cout << name << " STOPED" << std::endl;
  return true;
}

std::vector<std::string> getDiff(const std::string& param, int* res_code)
{
  std::vector<std::string> res;

  ontologenius::DifferenceFinder diff;
  const std::regex base_regex("(.*)\\|(.*)\\|(.*)");
  std::smatch base_match;
  if(std::regex_match(param, base_match, base_regex))
  {
    if(base_match.size() == 4)
    {
      const std::string onto1 = base_match[1].str();
      ontologenius::Ontology* onto1_ptr = nullptr;
      auto it = interfaces.find(onto1);
      if(it == interfaces.end())
      {
        *res_code = NO_EFFECT;
        return res;
      }
      else
        onto1_ptr = interfaces[onto1]->getOntology();

      const std::string onto2 = base_match[2].str();
      ontologenius::Ontology* onto2_ptr = nullptr;
      it = interfaces.find(onto2);
      if(it == interfaces.end())
      {
        *res_code = NO_EFFECT;
        return res;
      }
      else
        onto2_ptr = interfaces[onto2]->getOntology();

      const std::string concept = base_match[3].str();
      res = diff.getDiff(onto1_ptr, onto2_ptr, concept);
    }
  }
  else
    *res_code = UNKNOW_ACTION;

  return res;
}

bool managerHandle(ontologenius::compat::onto_ros::ServiceWrapper<ontologenius::compat::OntologeniusService::Request>& req,
                   ontologenius::compat::onto_ros::ServiceWrapper<ontologenius::compat::OntologeniusService::Response>& res)
{
  return [](auto&& req, auto&& res) {
    res->code = 0;

    removeUselessSpace(req->action);
    removeUselessSpace(req->param);

    if(req->action == "add")
    {
      auto it = interfaces.find(req->param);
      if(it != interfaces.end())
        res->code = NO_EFFECT;
      else
      {
        auto* tmp = new ontologenius::RosInterface(req->param);
        interfaces[req->param] = tmp;

        auto files = params.at("files").get();
        if((req->param.find("robot") != std::string::npos) && (params.at("robot_file").getFirst() != "none"))
          files.push_back(params.at("robot_file").getFirst());
        else if(params.at("human_file").getFirst() != "none")
          files.push_back(params.at("human_file").getFirst());

        tmp->setDisplay(params.at("display").getFirst() == "true");
        tmp->init(params.at("language").getFirst(),
                  params.at("intern_file").getFirst(),
                  files,
                  params.at("config").getFirst());

        std::thread th(&ontologenius::RosInterface::run, tmp);
        interfaces_threads[req->param] = std::move(th);

        std::cout << req->param << " STARTED" << std::endl;
      }
    }
    else if(req->action == "copy")
    {
      const std::regex base_regex("(.*)=(.*)");
      std::smatch base_match;
      if(std::regex_match(req->param, base_match, base_regex))
      {
        if(base_match.size() == 3)
        {
          const std::string base_name = base_match[2].str();
          const std::string copy_name = base_match[1].str();
          auto it = interfaces.find(base_name);
          if(it == interfaces.end()) // if interface to copy does not exist
            res->code = NO_EFFECT;
          else
          {
            auto* base_onto = it->second->getOntology();
            if(base_onto == nullptr)
              ontologenius::Display::error("You are trying to copy a malformed ontology");
            else if(!base_onto->isInit())
              ontologenius::Display::error("You are trying to copy an unclosed ontology ");
            else
            {
              it = interfaces.find(copy_name);
              if(it != interfaces.end()) // if copy already exist
                res->code = NO_EFFECT;
              else
              {
                auto* tmp = new ontologenius::RosInterface(*(interfaces[base_name]), copy_name);
                interfaces[copy_name] = tmp;
                tmp->setDisplay(params.at("display").getFirst() == "true");
                tmp->init(params.at("language").getFirst(),
                          params.at("config").getFirst());

                std::thread th(&ontologenius::RosInterface::run, tmp);
                interfaces_threads[copy_name] = std::move(th);

                std::cout << copy_name << " STARTED" << std::endl;
              }
            }
          }
        }
      }
    }
    else if(req->action == "delete")
    {
      auto it = interfaces.find(req->param);
      if(it == interfaces.end())
        res->code = NO_EFFECT;
      else if(deleteInterface(req->param) == false)
        res->code = REQUEST_ERROR;
    }
    else if(req->action == "list")
    {
      for(auto& it : interfaces)
        res->values.push_back(it.first);
    }
    else if(req->action == "difference")
    {
      int code = 0;
      res->values = getDiff(req->param, &code);
      res->code = code;
    }
    else
      res->code = UNKNOW_ACTION;

    return true;
  }(ontologenius::compat::onto_ros::getServicePointer(req), ontologenius::compat::onto_ros::getServicePointer(res));
}

int main(int argc, char** argv)
{
  signal(SIGSEGV, handler);
  ontologenius::compat::onto_ros::Node::init(argc, argv, "ontologenius_multi");

  params.insert(ontologenius::Parameter("language", {"-l", "--lang"}, {"en"}));
  params.insert(ontologenius::Parameter("intern_file", {"-i", "--intern_file"}, {"none"}));
  params.insert(ontologenius::Parameter("config", {"-c", "--config"}, {"none"}));
  params.insert(ontologenius::Parameter("display", {"-d", "--display"}, {"true"}));
  params.insert(ontologenius::Parameter("human_file", {"-h", "--robot"}, {"none"}));
  params.insert(ontologenius::Parameter("robot_file", {"-r", "--human"}, {"none"}));
  params.insert(ontologenius::Parameter("files", {}));

  params.set(argc, argv);
  params.display();

  const ontologenius::compat::onto_ros::Service<ontologenius::compat::OntologeniusService> service("ontologenius/manage", managerHandle);
  ontologenius::compat::onto_ros::Node::get().spin();

  std::vector<std::string> interfaces_names;
  std::transform(interfaces.cbegin(), interfaces.cend(), std::back_inserter(interfaces_names), [](const auto& it) { return it.first; });

  for(auto& interfaces_name : interfaces_names)
    deleteInterface(interfaces_name);

  while(ontologenius::compat::onto_ros::Node::ok())
    usleep(1);

  ontologenius::compat::onto_ros::Node::shutdown();

  return 0;
}
