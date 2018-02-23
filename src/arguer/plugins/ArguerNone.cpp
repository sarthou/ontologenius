#include "ontoloGenius/arguer/plugins/ArguerNone.h"
#include <pluginlib/class_list_macros.h>

void ArguerNone::preReason()
{
  //put your pre-reasonning here
}

void ArguerNone::postReason()
{
  //put your post-reasonning here
}

std::string ArguerNone::getName()
{
  return "arguer none";
}

std::string ArguerNone::getDesciption()
{
  return "This is an arguer model to show how to create your own arguer plugin";
}

PLUGINLIB_EXPORT_CLASS(ArguerNone, ArguerInterface)
