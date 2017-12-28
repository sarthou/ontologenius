#ifndef ERROR_H
#define ERROR_H

#include <string>
#include <map>

#include <stdint.h>

#include "ontoloGenius/Code.h"

class Error
{
public:
  Error(Code* code) {code_ = code; }
  ~Error() {}

  void printError(size_t pose, std::string message);
  void printWarning(size_t pose, std::string message);
private:
  Code* code_;

  size_t getBeginOfLine(size_t line_nb);
  void printCursor(size_t pose);
  void printMessage(size_t pose, std::string message);
};

#endif
