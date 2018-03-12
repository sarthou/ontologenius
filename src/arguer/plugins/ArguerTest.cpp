#include "ontoloGenius/arguer/plugins/ArguerTest.h"
#include <pluginlib/class_list_macros.h>

void ArguerTest::preReason()
{
  //put your pre-reasonning here
}

void ArguerTest::postReason()
{
  //put your post-reasonning here
}

std::string ArguerTest::getName()
{
  return "arguer test";
}

std::string ArguerTest::getDesciption()
{
  return "This is a test for ontoloGUI with a second arguer";
}

PLUGINLIB_EXPORT_CLASS(ArguerTest, ArguerInterface)
