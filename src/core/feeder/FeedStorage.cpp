#include "ontoloGenius/core/feeder/FeedStorage.h"
#include <iostream>

FeedStorage::FeedStorage() : base_regex("^\\[(\\w+)\\](.*)\\|(.*)\\|(.*)$"),
                             simple_regex("^\\[(\\w+)\\](.*)\\|$")
{

}

void FeedStorage::add(std::string regex)
{
  std::smatch base_match;
  if (std::regex_match(regex, base_match, base_regex))
  {
    if (base_match.size() == 5)
    {
      std::string action = base_match[1].str();
      std::string indiv1 = base_match[2].str();
      std::string prop = base_match[3].str();
      std::string indiv2 = base_match[4].str();
      std::cout << action << " : " << indiv1 << " : " << prop << " : " << indiv2 << std::endl;
    }
  }
  else if(std::regex_match(regex, base_match, simple_regex))
  {
    if (base_match.size() == 3)
    {
      std::string action = base_match[1].str();
      std::string indiv1 = base_match[2].str();
      std::cout << action << " : " << indiv1 << std::endl;
    }
  }
  else
    std::cout << "data do not match" << std::endl;
}
