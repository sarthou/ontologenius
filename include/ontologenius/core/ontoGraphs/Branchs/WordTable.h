#ifndef ONTOLOGENIUS_WORDTABLE_H
#define ONTOLOGENIUS_WORDTABLE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace ontologenius {

  using index_t = int64_t;

  class WordTable
  {
  public:
    WordTable()
    {
      table_.emplace_back(""); // index 0 is reserved for the "no result index"
    }

    index_t add(const std::string& value)
    {
      table_.push_back(value);
      return (index_t)table_.size() - 1;
    }

    std::string& get(index_t index)
    {
      return table_[index];
    }

    std::string& operator[](index_t index)
    {
      return table_[index];
    }

    const std::string& operator[](index_t index) const
    {
      return table_[index];
    }

    void index2string(std::unordered_set<std::string>& res, const std::unordered_set<index_t>& base)
    {
      for(index_t i : base)
        res.insert(table_[i]);
    }

    size_t size() const
    {
      return table_.size();
    }

  private:
    std::vector<std::string> table_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_WORDTABLE_H
