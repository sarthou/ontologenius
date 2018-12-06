#include "ontoloGenius/core/feeder/Feeder.h"
#include <iostream>

void Feeder::run()
{
  std::queue<feed_t> feeds = feed_storage_.get();
  while(feeds.empty() == false)
  {
    feed_t feed = feeds.front();
    feeds.pop();
    std::cout << feed.from_ << " : " << feed.prop_ << " : " << feed.on_ << std::endl;
  }
}
