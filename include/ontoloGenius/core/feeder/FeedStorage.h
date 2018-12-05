#ifndef FEEDSTORAGE
#define FEEDSTORAGE

#include <regex>

class FeedStorage
{
public:
  FeedStorage();

  void add(std::string regex);

private:
  std::regex base_regex;
  std::regex simple_regex;
};

#endif //FEEDSTORAGE
