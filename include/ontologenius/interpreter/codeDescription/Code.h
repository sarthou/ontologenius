#ifndef ONTOLOGENIUS_CODE_H
#define ONTOLOGENIUS_CODE_H

#include <string>
#include <map>
#include <vector>
#include <stdint.h>

#include "ontologenius/interpreter/codeDescription/TextManipulator.h"

#include "ontologenius/interpreter/codeDescription/Types/VariablesType.h"
#include "ontologenius/interpreter/codeDescription/Types/StringType.h"
#include "ontologenius/interpreter/codeDescription/Syntax/Operators.h"
#include "ontologenius/interpreter/codeDescription/Syntax/IfelseCF.h"

namespace ontologenius {

struct CommentBlock_t
{
  std::string comment;
  LinesCounter lines_count;
};

class Code;
struct SubsectionBlock_t
{
  Code* subsection;
  LinesCounter lines_count;
};

class Code : public TextManipulator
{
public:
  Code(std::string code) : TextManipulator(code), operators_(&text) {}
  ~Code();

  std::map<std::string, CommentBlock_t> comments_;
  std::map<std::string, SubsectionBlock_t> subsections_;
  IfelseCF ifelse_;
  StringType strings_;
  VariablesType variables_;
  Operators operators_;

  size_t getLineNumber(size_t final_pose);
  size_t getNbOfSublines(size_t& current_pose, size_t stop = -1);

  void goToEffectiveCode(std::string& code, size_t& pose);
  void removeNonEffectiveCode(std::string& code);
  size_t checkWordIntegrity(std::string& wholeWord);
  void removeProtectedWord(std::string& text);

private:
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CODE_H
