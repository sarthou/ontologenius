#ifndef IFELSECF_H
#define IFELSECF_H

#include <string>
#include <map>
#include <stdint.h>

#include "ontoloGenius/interpreter/codeDescription/LinesCounter.h"

namespace ontologenius {

struct IfBlock_t
{
  std::string IfBlock_condition;
  size_t cond_pose;
  std::string IfBlock_if;
  size_t if_pose;
  std::string IfBlock_else;
  size_t else_pose;
  LinesCounter lines_count;
};

class Code;
class Error;

class IfelseCF
{
public:
  IfelseCF() {}
  ~IfelseCF() {}

  void compact(Code& code, Error* error = nullptr);
  bool uncompact(Code& code);

  IfBlock_t &operator[] (std::string index) {return ifelse_[index]; }

  std::map<std::string, std::string> ifelse_code_;

private:
  std::map<std::string, IfBlock_t> ifelse_;

  size_t getNextIfBlock(int& nb_block, size_t pose, Code& code, Error* error = nullptr);
  size_t getCorrectCharPosition(std::string text, size_t pose);
};

} // namespace ontologenius

#endif
