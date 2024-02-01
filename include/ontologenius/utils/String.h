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
        res.emplace_back(part);
    }
    res.emplace_back(tmp_text);
    return res;
  }

  inline size_t getIn(size_t begin, std::string& in_bracket, const std::string& text, char symbol_in, char symbol_out)
  {
    size_t pose = begin;

    if(text.at(pose) == symbol_in)
    {
      size_t first_pose = pose;
      int cpt = 1;
      while((cpt != 0) && (pose+1 < text.length()))
      {
        ++pose;
        if(text.at(pose) == symbol_in)
          cpt++;
        else if(text.at(pose) == symbol_out)
          cpt--;

      }

      in_bracket = text.substr(first_pose+1, pose-first_pose-1);

      if(cpt == 0)
        return pose;
      else
        return std::string::npos;
    }
    else
      return begin;
  }

  inline bool isIn(const std::string& substring, const std::string& string)
  {
    return (string.find(substring) != std::string::npos);
  }
} // namespace ontologenius

#endif // ONTOLOGENIUS_STRING_H
