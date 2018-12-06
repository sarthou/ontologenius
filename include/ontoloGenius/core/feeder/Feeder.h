#ifndef FEEDER_H
#define FEEDER_H

#include "ontoloGenius/core/feeder/FeedStorage.h"

class Feeder
{
public:
  Feeder() {}

  void store(std::string feed) { feed_storage_.add(feed); }
  void run();

private:
  FeedStorage feed_storage_;
};

#endif
