#include "ontologenius/core/feeder/Versionor.h"

#include <iostream>

using namespace ontologenius;

feed_t creatFeed(const std::string& data)
{
  feed_t feed;
  feed.action_ = action_add;
  feed.from_ = data;

  return feed;
}

int main()
{
  Versionor version(nullptr);

  version.insert(creatFeed("a"));
  version.insert(creatFeed("b"));
  version.commit("1");

  version.insert(creatFeed("c"));
  version.insert(creatFeed("d"));
  version.commit("2");

  std::cout << "checkout 1" << std::endl;
  version.checkout("1");

  version.insert(creatFeed("e"));
  version.commit("3");

  std::cout << "checkout 2" << std::endl;
  version.checkout("2");

  version.insert(creatFeed("f"));
  version.commit("4");

  std::cout << "checkout 3" << std::endl;
  version.checkout("3");

  version.insert(creatFeed("g"));
  version.insert(creatFeed("h"));
  version.commit("5");

  std::cout << "checkout 0" << std::endl;
  version.checkout("0");

  version.insert(creatFeed("i"));
  version.insert(creatFeed("j"));
  version.commit("6");

  version.print();
  version.exportToXml("");

  return 0;
}
