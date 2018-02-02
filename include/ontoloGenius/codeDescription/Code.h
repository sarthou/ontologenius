#ifndef CODE_H
#define CODE_H

#include <string>
#include <map>
#include <vector>

#include <iostream>

#include <stdint.h>

#include "ontoloGenius/codeDescription/TextManipulator.h"

#include "ontoloGenius/codeDescription/Types/VariablesType.h"
#include "ontoloGenius/codeDescription/Types/StringType.h"
#include "ontoloGenius/codeDescription/Operators.h"

struct IfBlock_t
{
  std::string IfBlock_condition;
  size_t cond_pose;
  std::string IfBlock_if;
  size_t if_pose;
  std::string IfBlock_else;
  size_t else_pose;
  LinesCounter lines_count;
  std::string initial_code;
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

class Code : public TextManipulator
{
public:
  Code(std::string code) : TextManipulator(code), operators_(&text) {}
  ~Code() {}

  std::string text;
  LinesCounter lines_counter_;

  std::map<std::string, CommentBlock_t> comments_;
  std::map<std::string, SubsectionBlock_t> subsections_;
  std::map<std::string, IfBlock_t> ifelse_;
  StringType strings_;
  VariablesType variables_;
  Operators operators_;

  size_t getLineNumber(size_t final_pose);
  size_t getNbOfSublines(size_t& current_pose, size_t stop = -1);

  void goToEffectiveCode(std::string& code, size_t& pose);

private:
};

#endif
