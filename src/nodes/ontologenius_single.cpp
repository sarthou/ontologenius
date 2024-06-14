#include <array>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <ontologenius/compat/ros.h>
#include <thread>
#include <unistd.h>

#include "ontologenius/interface/RosInterface.h"
#include "ontologenius/utils/Parameters.h"

void handler(int sig)
{
  std::array<void*, 10> array;
  auto size = backtrace(array.data(), 10);

  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array.data(), size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char** argv)
{
  signal(SIGSEGV, handler);

  ontologenius::compat::onto_ros::Node::init(argc, argv, "ontologenius_single");

  std::thread th([]() { ontologenius::compat::onto_ros::Node::get().spin(); });

  {
    ontologenius::RosInterface interface;

    ontologenius::Parameters params;
    params.insert(ontologenius::Parameter("language", {"-l", "--lang"}, {"en"}));
    params.insert(ontologenius::Parameter("intern_file", {"-i", "--intern_file"}, {"none"}));
    params.insert(ontologenius::Parameter("config", {"-c", "--config"}, {"none"}));
    params.insert(ontologenius::Parameter("display", {"-d", "--display"}, {"true"}));
    params.insert(ontologenius::Parameter("files", {}));

    params.set(argc, argv);
    params.display();

    interface.setDisplay(params.at("display").getFirst() == "true");
    interface.init(params.at("language").getFirst(),
                   params.at("intern_file").getFirst(),
                   params.at("files").get(),
                   params.at("config").getFirst());

    interface.run();
  }

  ontologenius::compat::onto_ros::Node::shutdown();

  th.join();

  return 0;
}
