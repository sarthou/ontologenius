#ifndef ONTOLOGENIUS_FEEDSTORAGE_H
#define ONTOLOGENIUS_FEEDSTORAGE_H

#include <regex>
#include <mutex>
#include <string>
#include <queue>

namespace ontologenius {

enum action_t
{
  action_add,
  action_del,
  action_commit,
  action_checkout,
  action_nop
};

struct feed_t
{
  action_t action_;
  std::string from_;
  std::string prop_;
  std::string on_;
  bool checkout_;

  feed_t() : action_(action_nop), checkout_(false) {}
};

class FeedStorage
{
public:
  FeedStorage();

  void add(const std::string& regex);
  void add(std::vector<feed_t>& datas);
  std::queue<feed_t> get();
  size_t size() { return fifo_1.size() + fifo_2.size(); }

private:
  std::regex base_regex;
  std::regex simple_regex;

  std::mutex mutex_;

  bool queue_choice_;
  std::queue<feed_t> fifo_1;
  std::queue<feed_t> fifo_2;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDSTORAGE_H
