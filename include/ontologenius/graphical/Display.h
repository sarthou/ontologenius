#ifndef ONTOLOGENIUS_DISPLAY_H
#define ONTOLOGENIUS_DISPLAY_H

#include <iostream>

#include "ontologenius/graphical/color.h"

namespace ontologenius {

  /// @brief This is a class with static methods to display information to the user with different levels
  class Display
  {
  public:
    /// @brief Displays a message in blue
    /// @param text is the string to display
    /// @param new_line can be setted to false to not return a the line
    static void info(const std::string& text, bool new_line = true)
    {
      std::cout << COLOR_BLUE << text << COLOR_OFF;
      if(new_line)
        std::cout << std::endl;
    }

    /// @brief Displays a message in orange
    /// @param text is the string to display
    /// @param new_line can be setted to false to not return a the line
    static void warning(const std::string& text, bool new_line = true)
    {
      std::cout << COLOR_ORANGE << text << COLOR_OFF;
      if(new_line)
        std::cout << std::endl;
    }

    /// @brief Displays a message in red
    /// @param text is the string to display
    /// @param new_line can be setted to false to not return a the line
    static void error(const std::string& text, bool new_line = true)
    {
      std::cout << COLOR_RED << text << COLOR_OFF;
      if(new_line)
        std::cout << std::endl;
    }

    /// @brief Displays a message in green
    /// @param text is the string to display
    /// @param new_line can be setted to false to not return a the line
    static void success(const std::string& text, bool new_line = true)
    {
      std::cout << COLOR_GREEN << text << COLOR_OFF;
      if(new_line)
        std::cout << std::endl;
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_DISPLAY_H
