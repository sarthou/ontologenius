#include "ontologenius/core/reasoner/plugins/ReasonerNone.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

void ReasonerNone::preReason()
{
  //put your pre-reasonning here
}

void ReasonerNone::postReason()
{
  //put your post-reasonning here
}

void ReasonerNone::periodicReason()
{
  //put your periodic reasonning here
}

std::string ReasonerNone::getName()
{
  return "reasoner none";
}

std::string ReasonerNone::getDesciption()
{
  return "This is an reasoner model to show how to create your own reasoner plugin";
}

PLUGINLIB_EXPORT_CLASS(ReasonerNone, ReasonerInterface)

} // namespace ontologenius
