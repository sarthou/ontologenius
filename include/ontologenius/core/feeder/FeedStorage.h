#ifndef ONTOLOGENIUS_FEEDSTORAGE_H
#define ONTOLOGENIUS_FEEDSTORAGE_H

#include <mutex>
#include <queue>
#include <regex>
#include <string>

namespace ontologenius {

  enum action_e
  {
    action_add,
    action_del,
    action_commit,
    action_checkout,
    action_nop
  };

  struct RosTime_t
  {
    uint32_t sec;
    uint32_t nsec;

    RosTime_t() : sec(0), nsec(0) {}
    RosTime_t(uint32_t seconds, uint32_t n_seconds) : sec(seconds), nsec(n_seconds) {}
  };

  struct feed_t
  {
    action_e action_;
    RosTime_t stamp;
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

    void add(const std::string& regex, const RosTime_t& stamp);
    void add(std::vector<feed_t>& datas);
    std::queue<feed_t> get();
    size_t size() { return fifo_1_.size() + fifo_2_.size(); }

  private:
    std::regex base_regex_;
    std::regex simple_regex_;

    std::mutex mutex_;

    bool queue_choice_;
    std::queue<feed_t> fifo_1_;
    std::queue<feed_t> fifo_2_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDSTORAGE_H
