#include <ros/ros.h>

#include "ontologenius/utils/Parameters.h"
#include "ontologenius/RosInterface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius");

  ros::NodeHandle n;

  ontologenius::RosInterface interface(&n);

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

  return 0;
}
