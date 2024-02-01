#include "ontologenius/core/reasoner/plugins/ReasonerNone.h"

#include <pluginlib/class_list_macros.hpp>

namespace ontologenius {

bool ReasonerNone::preReason(const QueryInfo_t& query_info)
{
  //put your pre-reasonning here
  (void)query_info;
  return false;
}

void ReasonerNone::postReason()
{
  //put your post-reasonning here
}

bool ReasonerNone::periodicReason()
{
  //put your periodic reasonning here
  return false;
}

std::string ReasonerNone::getName()
{
  return "reasoner none";
}

std::string ReasonerNone::getDescription()
{
  return "This is an reasoner model to show how to create your own reasoner plugin";
}

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerNone, ontologenius::ReasonerInterface)
