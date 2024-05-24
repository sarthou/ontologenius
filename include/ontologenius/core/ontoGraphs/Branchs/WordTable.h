#ifndef ONTOLOGENIUS_WORDTABLE_H
#define ONTOLOGENIUS_WORDTABLE_H

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace ontologenius {

  typedef int64_t index_t;

  class WordTable
  {
  public:
    WordTable()
    {
      table_.push_back(""); // index 0 is reserved for the "no result index"
    }

    inline index_t add(const std::string& value)
    {
      table_.push_back(value);
      return table_.size() - 1;
    }

    inline std::string& get(index_t index)
    {
      return table_[index];
    }

    inline std::string& operator[](index_t index)
    {
      return table_[index];
    }

    inline const std::string& operator[](index_t index) const
    {
      return table_[index];
    }

    inline void index2string(std::unordered_set<std::string>& res, const std::unordered_set<index_t>& base)
    {
      for(index_t i : base)
        res.insert(table_[i]);
    }

    inline size_t size() const
    {
      return table_.size();
    }

  private:
    std::vector<std::string> table_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_WORDTABLE_H
