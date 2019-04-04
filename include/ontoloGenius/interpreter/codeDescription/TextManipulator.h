#ifndef TEXTMANIPULATOR_H
#define TEXTMANIPULATOR_H

#include <string>

#include <iostream>

#include <stdint.h>

#include "ontoloGenius/interpreter/codeDescription/LinesCounter.h"

namespace ontologenius {

class TextManipulator
{
public:
  TextManipulator(std::string textual) {text = textual;}
  ~TextManipulator() {}

  std::string text;
  LinesCounter lines_counter_;
  size_t first_char_pose;

  size_t getInBraquet(size_t begin, std::string& in_bracket, std::string& text);
  bool findBefore(size_t begin, char symbol);
  bool findJustBefore(size_t begin, std::string symbol);
  std::string getWordBefore(size_t begin);
  std::string getWordAfter(size_t begin, bool just_after = true);
  size_t findAfter(size_t begin, std::string symbol);
  bool findHere(size_t begin, std::string symbol);

  void print() {std::cout << text << std::endl; }
  void remove(char character);

private:
};

} // namespace ontologenius

#endif
