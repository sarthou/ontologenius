#include "ontologenius/core/subscription/SubscriptionPattern.h"

#include <regex>

namespace ontologenius {

  std::regex SubscriptionPattern::regex(R"((\w)\|(\w+)\|(\w+)\|(\w+))");
  std::regex SubscriptionPattern::regex2(R"(\[([^\]]+)\]([^|]+)\|([^|]+)\|([^|]+))");
  std::smatch SubscriptionPattern::match;

} // namespace ontologenius
