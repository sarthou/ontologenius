#ifndef ONTOLOGENIUS_STRING_H
#define ONTOLOGENIUS_STRING_H

#include <string>
#include <vector>

namespace ontologenius {

  inline std::vector<std::string> split(const std::string& text, const std::string& delim)
  {
    std::vector<std::string> res;
    std::string tmp_text = text;
    while(tmp_text.find(delim) != std::string::npos)
    {
      size_t pos = tmp_text.find(delim);
      std::string part = tmp_text.substr(0, pos);
      tmp_text = tmp_text.substr(pos + delim.size(), tmp_text.size() - pos - delim.size());
      if(part != "")
        res.push_back(part);
    }
    res.push_back(tmp_text);
    return res;
  }

} // namespace ontologenius

#endif // ONTOLOGENIUS_STRING_H
