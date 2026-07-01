#ifndef ONTOLOGENIUS_INTERFACEPARAMS_H
#define ONTOLOGENIUS_INTERFACEPARAMS_H

#include <cerrno>
#include <climits>
#include <cstdlib>
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

    InterfaceParams() : main_index(0), optional_index(0),
                        depth(-1), selector_index(0),
                        threshold(-1), take_id(true) {}

    std::string operator()() const { return base; }

    void extractStringParams(const std::string& param)
    {
      std::vector<std::string> str_params = split(param, " ");

      if(str_params.empty() == false)
        base = str_params[0];

      for(size_t i = 1; i < str_params.size(); i++)
      {
        if((str_params[i] == "-d") || (str_params[i] == "--depth"))
        {
          i++;
          int tmp = -1;
          if(parseInt(str_params[i], tmp) == false)
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
          double tmp = -1;
          if(parseDouble(str_params[i], tmp) == false)
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
        else if((str_params[i].empty() == false) && (str_params[i][0] == '-'))
        {
          Display::warning("[WARNING] unknow parameter \"" + str_params[i] + "\"");
          i++;
        }
        else if(str_params[i].empty() == false)
          base += " " + str_params[i];
      }
    }

    void extractIndexParams(const std::string& param)
    {
      std::vector<std::string> str_params = split(param, " ");

      if(str_params.empty() == false)
        base = str_params[0];

      if((str_params.empty() == true) || (parseIndexPair(str_params[0], main_index, optional_index) == false))
        main_index = 0;

      for(size_t i = 1; i < str_params.size(); i++)
      {
        if((str_params[i] == "-d") || (str_params[i] == "--depth"))
        {
          i++;
          int tmp = -1;
          if(parseInt(str_params[i], tmp) == false)
            tmp = -1;
          depth = tmp;
        }
        else if((str_params[i] == "-s") || (str_params[i] == "--selector"))
        {
          i++;
          selector = str_params[i];
          if(parseInt64(str_params[i], selector_index) == false)
            selector_index = 0;
        }
        else if((str_params[i] == "-t") || (str_params[i] == "--threshold"))
        {
          i++;
          double tmp = -1;
          if(parseDouble(str_params[i], tmp) == false)
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
        else if((str_params[i].empty() == false) && str_params[i][0] == '-')
        {
          Display::warning("[WARNING] unknow parameter \"" + str_params[i] + "\"");
          i++;
        }
        else if(str_params[i].empty() == false)
          base += " " + str_params[i];
      }
    }

  private:
    static bool parseInt(const std::string& str, int& value)
    {
      errno = 0;
      char* end = nullptr;
      const long parsed = std::strtol(str.c_str(), &end, 10);
      if((end == str.c_str()) || (*end != '\0') || (errno != 0) || (parsed < INT_MIN) || (parsed > INT_MAX))
        return false;
      value = static_cast<int>(parsed);
      return true;
    }

    static bool parseInt64(const std::string& str, int64_t& value)
    {
      errno = 0;
      char* end = nullptr;
      const long long parsed = std::strtoll(str.c_str(), &end, 10);
      if((end == str.c_str()) || (*end != '\0') || (errno != 0))
        return false;
      value = static_cast<int64_t>(parsed);
      return true;
    }

    static bool parseDouble(const std::string& str, double& value)
    {
      errno = 0;
      char* end = nullptr;
      const double parsed = std::strtod(str.c_str(), &end);
      if((end == str.c_str()) || (*end != '\0') || (errno != 0))
        return false;
      value = parsed;
      return true;
    }

    static bool parseIndexPair(const std::string& str, int64_t& first, int64_t& second)
    {
      errno = 0;
      char* end = nullptr;
      const long long parsed_first = std::strtoll(str.c_str(), &end, 10);
      if((end == str.c_str()) || (errno != 0))
        return false;

      first = static_cast<int64_t>(parsed_first);

      if(*end == ':')
      {
        errno = 0;
        char* end_second = nullptr;
        const long long parsed_second = std::strtoll(end + 1, &end_second, 10);
        if((end_second == end + 1) || (*end_second != '\0') || (errno != 0))
          return true;
        second = static_cast<int64_t>(parsed_second);
        return true;
      }

      return (*end == '\0');
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_INTERFACEPARAMS_H