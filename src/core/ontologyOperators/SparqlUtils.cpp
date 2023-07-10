#include "ontologenius/core/ontologyOperators/SparqlUtils.h"

#include <algorithm>

namespace ontologenius
{
  template<> index_t getDefaultSelector<index_t>() { return 0; }
  bool isSelectorDefined(const std::string& value) { return (value != ""); }

  template<>
  index_t convertResourceValue<index_t>(const std::string& value)
  {
    try {
      return std::stoll(value);
    }
    catch(...) {
      return 0;
    }
  }

  void removeUselessSpace(std::string& text)
  {
    while((text[0] == ' ') && (text.size() != 0))
      text.erase(0,1);

    while((text[text.size() - 1] == ' ') && (text.size() != 0))
      text.erase(text.size() - 1,1);
  }

  void removeChar(std::string& text, const std::vector<char>& delim)
  {
    for(auto c : delim)
      text.erase(std::remove(text.begin(), text.end(), c), text.end());
  }

} // namespace ontologenius