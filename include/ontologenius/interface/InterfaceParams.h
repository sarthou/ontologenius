#ifndef ONTOLOGENIUS_INTERFACEPARAMS_H
#define ONTOLOGENIUS_INTERFACEPARAMS_H

#include <string>
#include <vector>

#include "ontologenius/graphical/Display.h"

#include "ontologenius/utils/String.h"

namespace ontologenius {

class InterfaceParams
{
public:
  std::string base;

  int64_t main_index;
  int64_t optional_index;

  size_t depth;
  std::string selector;
  int64_t selector_index;
  double threshold;
  bool take_id;

  InterfaceParams(): main_index(0), optional_index(0), depth(-1), selector_index(0), threshold(-1), take_id(true) {}

  std::string operator()() { return base; }

  void extractStringParams(const std::string& param)
  {
    std::vector<std::string> str_params = split(param, " ");

    if(str_params.size())
      base = str_params[0];

    for(size_t i = 1; i < str_params.size(); i++)
    {
      if((str_params[i] == "-d") || (str_params[i] == "--depth"))
      {
        i++;
        int tmp = -1;
        if(sscanf(str_params[i].c_str(), "%d", &tmp) != 1)
          tmp = -1;
        depth = tmp;
      }
      else if((str_params[i] == "-s") || (str_params[i] == "--selector"))
      {
        i++;
        selector = str_params[i];
      }
      else if((str_params[i] == "-t") || (str_params[i] == "--threshold"))
      {
        i++;
        float tmp = -1;
        if(sscanf(str_params[i].c_str(), "%f", &tmp) != 1)
          tmp = -1;
        threshold = tmp;
      }
      else if((str_params[i] == "-i") || (str_params[i] == "--take_id"))
      {
        i++;
        bool tmp = false;
        if(str_params[i] == "true")
          tmp = true;

        take_id = tmp;
      }
      else if(str_params[i].size() && str_params[i][0] == '-')
      {
        Display::warning("[WARNING] unknow parameter \"" + str_params[i] + "\"");
        i++;
      }
      else if(str_params[i].size())
        base += " " + str_params[i];
    }
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INTERFACEPARAMS_H