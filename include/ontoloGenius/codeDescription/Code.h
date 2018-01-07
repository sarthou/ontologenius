#ifndef CODE_H
#define CODE_H

#include <string>
#include <map>
#include <vector>

#include <iostream>

#include <stdint.h>

#include "ontoloGenius/codeDescription/Variables.h"
#include "ontoloGenius/codeDescription/String.h"
#include "ontoloGenius/codeDescription/Operators.h"

struct IfBlock_t
{
  std::string IfBlock_condition;
  std::string IfBlock_if;
  std::string IfBlock_else;
  LinesCounter lines_count;
};

struct CommentBlock_t
{
  std::string comment;
  LinesCounter lines_count;
};

struct SubsectionBlock_t
{
  std::string subsection;
  LinesCounter lines_count;
};

class Code
{
public:
  Code(std::string code) : operators_(&text) {text = code; }
  ~Code() {}

  std::string text;
  LinesCounter lines_counter_;

  std::map<std::string, CommentBlock_t> comments_;
  std::map<std::string, SubsectionBlock_t> subsections_;
  std::map<std::string, IfBlock_t> ifelse_;
  String strings_;
  Variables variables_;
  Operators operators_;

  size_t getInBraquet(size_t begin, std::string& in_braquet, std::string& text);
  bool findBefore(size_t begin, char symbol);
  bool findJustBefore(size_t begin, std::string symbol);
  std::string getWordBefore(size_t begin);
  std::string getWordAfter(size_t begin);
  size_t findAfter(size_t begin, std::string symbol);
  bool findHere(size_t begin, std::string symbol);

  size_t getLineNumber(size_t final_pose);
  size_t getNbOfSublines(size_t& current_pose, size_t stop = -1);

  void print() {std::cout << text << std::endl; }
  void remove(char character);

private:
};

#endif
