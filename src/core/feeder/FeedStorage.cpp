#include "ontologenius/core/feeder/FeedStorage.h"

#include <iostream>

namespace ontologenius {

FeedStorage::FeedStorage() : base_regex(R"(^\[(\w+)\](.*)\|(.*)\|(.*)$)"),
                             simple_regex(R"(^\[(\w+)\](.*)\|$)")
{
  queue_choice_ = true;
}

void FeedStorage::add(const std::string& regex)
{
  std::smatch base_match;
  feed_t feed;
  if (std::regex_match(regex, base_match, base_regex))
  {
    if (base_match.size() == 5)
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
  else if(std::regex_match(regex, base_match, simple_regex))
  {
    if (base_match.size() == 3)
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
    fifo_1.push(feed);
  else
    fifo_2.push(feed);
  mutex_.unlock();
}

void FeedStorage::add(std::vector<feed_t>& datas)
{
  mutex_.lock();
  if(queue_choice_ == true)
  {
    for(auto& data : datas)
      fifo_1.push(data);
  }
  else
  {
    for(auto& data : datas)
      fifo_2.push(data);
  }
  mutex_.unlock();
}

std::queue<feed_t> FeedStorage::get()
{
  std::queue<feed_t> tmp;
  mutex_.lock();
  if(queue_choice_ == true)
  {
    while(!fifo_2.empty())
      fifo_2.pop();
    queue_choice_ = false;
    tmp = fifo_1;
  }
  else
  {
    while(!fifo_1.empty())
      fifo_1.pop();
    queue_choice_ = true;
    tmp = fifo_2;
  }
  mutex_.unlock();
  return tmp;
}

} // namespace ontologenius
