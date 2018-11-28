#include "ontoloGenius/core/arguer/plugins/ArguerGeneralize.h"
#include <pluginlib/class_list_macros.h>

void ArguerGeneralize::preReason()
{
  //put your pre-reasonning here
}

void ArguerGeneralize::postReason()
{
  //put your post-reasonning here
}

std::string ArguerGeneralize::getName()
{
  return "arguer generalize";
}

std::string ArguerGeneralize::getDesciption()
{
  return "This arguer aims to infer new knowledge by generalizing explicit relationships between concepts.";
}

PLUGINLIB_EXPORT_CLASS(ArguerGeneralize, ArguerInterface)
