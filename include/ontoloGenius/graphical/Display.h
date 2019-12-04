#ifndef ONTOLOGENIUS_DISPLAY_H
#define ONTOLOGENIUS_DISPLAY_H

#include <sstream>
#include "ontoloGenius/graphical/color.h"

namespace ontologenius
{

class Display
{
public:

  static void info(const std::string& text, bool new_line = true)
  {
    std::cout << COLOR_BLUE << text << COLOR_OFF;
    if(new_line)
      std::cout << std::endl;
  }

  static void warning(const std::string& text, bool new_line = true)
  {
    std::cout << COLOR_ORANGE << text << COLOR_OFF;
    if(new_line)
      std::cout << std::endl;
  }

  static void error(const std::string& text, bool new_line = true)
  {
    std::cout << COLOR_RED << text << COLOR_OFF;
    if(new_line)
      std::cout << std::endl;
  }

  static void success(const std::string& text, bool new_line = true)
  {
    std::cout << COLOR_GREEN << text << COLOR_OFF;
    if(new_line)
      std::cout << std::endl;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DISPLAY_H
