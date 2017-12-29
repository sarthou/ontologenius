#ifndef STRING_H
#define STRING_H

#include <string>
#include <map>

#include "ontoloGenius/codeDescription/LinesCounter.h"
#include "ontoloGenius/codeDescription/FunctionContainer.h"

struct StringBlock_t
{
  std::string strings;
  LinesCounter lines_count;
};

class String : public FunctionContainer
{
public:
  String();
  ~String() {};

  std::string add(std::string text, size_t line_start, size_t line_stop);
  std::string get(std::string id);
  size_t nbLines(std::string id);

  size_t size(std::string id);

private:
  uint16_t nb_strings_;

  std::map <std::string, StringBlock_t> strings_;
};

#endif
