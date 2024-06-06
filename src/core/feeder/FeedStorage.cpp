#include "ontologenius/core/feeder/FeedStorage.h"

#include <algorithm>
#include <iostream>
#include <queue>
#include <regex>
#include <string>
#include <vector>

namespace ontologenius {

  FeedStorage::FeedStorage() : base_regex_(R"(^\[(\w+)\](.*)\|(.*)\|(.*)$)"),
                               simple_regex_(R"(^\[(\w+)\](.*)\|$)")
  {
    queue_choice_ = true;
  }

  void FeedStorage::add(const std::string& regex, const RosTime_t& stamp)
  {
    std::smatch base_match;
    Feed_t feed;
    feed.stamp = stamp;
    if(std::regex_match(regex, base_match, base_regex_))
    {
      if(base_match.size() == 5)
      {
        std::string action = base_match[1].str();
        std::transform(action.begin(), action.end(), action.begin(), ::tolower);
        if(action == "add")
          feed.action_ = action_add;
        else if(action == "del")
          feed.action_ = action_del;
        else if(action == "nop")
          feed.action_ = action_nop;
        else
        {
          std::cout << "data do not match" << std::endl;
          return;
        }
        feed.from_ = base_match[2].str();
        feed.prop_ = base_match[3].str();
        feed.on_ = base_match[4].str();
      }
    }
    else if(std::regex_match(regex, base_match, simple_regex_))
    {
      if(base_match.size() == 3)
      {
        std::string action = base_match[1].str();
        std::transform(action.begin(), action.end(), action.begin(), ::tolower);
        if(action == "add")
          feed.action_ = action_add;
        else if(action == "del")
          feed.action_ = action_del;
        else if(action == "commit")
          feed.action_ = action_commit;
        else if(action == "checkout")
          feed.action_ = action_checkout;
        else if(action == "nop")
          feed.action_ = action_nop;
        else
        {
          std::cout << "data do not match" << std::endl;
          return;
        }
        feed.from_ = base_match[2].str();
      }
    }
    else if(regex == "[nop]")
      feed.action_ = action_nop;
    else
    {
      std::cout << "data do not match" << std::endl;
      return;
    }

    mutex_.lock();
    if(queue_choice_ == true)
      fifo_1_.push(feed);
    else
      fifo_2_.push(feed);
    mutex_.unlock();
  }

  void FeedStorage::add(std::vector<Feed_t>& datas)
  {
    mutex_.lock();
    if(queue_choice_ == true)
    {
      for(auto& data : datas)
        fifo_1_.push(data);
    }
    else
    {
      for(auto& data : datas)
        fifo_2_.push(data);
    }
    mutex_.unlock();
  }

  std::queue<Feed_t> FeedStorage::get()
  {
    std::queue<Feed_t> tmp;
    mutex_.lock();
    if(queue_choice_ == true)
    {
      fifo_2_ = std::queue<Feed_t>();
      queue_choice_ = false;
      tmp = std::move(fifo_1_);
      fifo_1_ = std::queue<Feed_t>();
    }
    else
    {
      fifo_1_ = std::queue<Feed_t>();
      queue_choice_ = true;
      tmp = std::move(fifo_2_);
      fifo_2_ = std::queue<Feed_t>();
    }
    mutex_.unlock();
    return tmp;
  }

} // namespace ontologenius
