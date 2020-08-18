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
  params.insert(ontologenius::Parameter("files", {}));

  params.set(argc, argv);
  params.display();

  interface.init(params.parameters_.at("language").getFirst(),
                 params.parameters_.at("intern_file").getFirst(),
                 params.parameters_.at("files").get(),
                 params.parameters_.at("config").getFirst());
  interface.run();

  return 0;
}
