#ifndef ONTOLOGENIUS_STRING_H
#define ONTOLOGENIUS_STRING_H

#include <string>
#include <map>

#include "ontoloGenius/interpreter/codeDescription/LinesCounter.h"
#include "ontoloGenius/interpreter/codeDescription/Functions/FunctionContainer.h"

namespace ontologenius {

struct StringBlock_t
{
  std::string strings;
  LinesCounter lines_count;
};

class StringType : public FunctionContainer
{
public:
  StringType();
  ~StringType() {};

  std::string add(std::string text, size_t line_start, size_t line_stop);
  std::string get(std::string id);
  size_t nbLines(std::string id);

  size_t size(std::string id);

private:
  uint16_t nb_strings_;

  std::map <std::string, StringBlock_t> strings_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_STRING_H
