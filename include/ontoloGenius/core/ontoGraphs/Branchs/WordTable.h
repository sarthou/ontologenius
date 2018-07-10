#ifndef WORDTABLE_H
#define WORDTABLE_H

#include <vector>
#include <string>
#include <unordered_set>

class WordTable
{
public:
  WordTable() {}
  ~WordTable() {}

  inline unint32_t add(const std::string& value)
  {
    table_.push_back(value);
    return table_.size() - 1;
  }

  inline std::string& get(unint32_t index)
  {
    return table_[index];
  }

  inline void index2string(std::unordered_set<std::string>& res, std::unordered_set<unint32_t>& base)
  {
    for(unint32_t i : base)
      res.insert(table_[i]);
  }
private:
  std::vector<std::string>> table_;
};

#endif
