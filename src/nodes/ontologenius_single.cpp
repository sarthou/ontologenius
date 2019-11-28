#include <ros/ros.h>

#include "ontoloGenius/RosInterface.h"
#include "ontoloGenius/Parameters.h"

#include "ontoloGenius/core/Computer.h"
#include "ontoloGenius/interpreter/Parser.h"
//#define USE_INTEPRETER

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius");

  ros::NodeHandle n;

  ros::service::waitForService("ontologenius/rest", -1);

  ontologenius::RosInterface interface(&n);

  ontologenius::Parameters params;
  params.insert(ontologenius::Parameter("language", {"-l", "--lang"}, {"en"}));
  params.insert(ontologenius::Parameter("intern_file", {"-i", "--intern_file"}, {"none"}));
  params.insert(ontologenius::Parameter("files", {}));

  params.set(argc, argv);
  params.display();

  interface.init(params.parameters_.at("language").getFirst(),
                 params.parameters_.at("intern_file").getFirst(),
                 params.parameters_.at("files").get());
  interface.run();

  ROS_DEBUG("KILL ontoloGenius");

  #ifdef USE_INTEPRETER
    std::string code = "";
    code += "var::man += fablab.isIn() - (bob + max);\n";
    code += "var::man.toString();\n";
    code += "if(adult == age) \n";
    code += "{";
    code += "//this is a comment\n";
    code += "\tif(age == adult)\n";
    code += "\t\tont::print(var::man);\n";
    code += "}\n";
    code += "else if(old == /* */age)\n";
    code += "\tif(young == age)\n";
    code += "\t\tont::print(\"this is an else if\");\n";
    code += "/*\n";
    code += "an other comment*/ \n";
    code += "ont::print(\"this is the else \");\n\n";
    code += "ont::null();\n";
    code += "var::men =if(var::man == man);\n";

    ontologenius::Error error;

    ontologenius::Code my_code(code);
    ontologenius::Parser p(&my_code);

    error.cpy(p.getError());
    error.printStatus();
  #endif

  return 0;
}

/*else if(req.action == "test")
{
  Computer comp;
  res.values.resize(1);
  if(comp.compute(req.param, onto->class_graph_))
    res.values[0] = "true";
  else
    res.values[0] = "false";
  //comp.compute("red_cube|young_animal=color_animal|age_object", onto);
}*/
