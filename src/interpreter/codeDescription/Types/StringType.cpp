#include "ontoloGenius/interpreter/codeDescription/Types/StringType.h"

StringType::StringType()
{
  nb_strings_ = 0;

  FunctionDescriptor op_add = FunctionDescriptor("opAdd", type_string, std::vector<type_t>(1, type_string));
  op_add.overload(type_string, std::vector<type_t>(1, type_word));
  op_add.overload(type_string, std::vector<type_t>(1, type_word_set));
  op_add.addExplicitName("addition operator");
  functions_.push_back(op_add);

  FunctionDescriptor op_sub = FunctionDescriptor("opSub", type_string, std::vector<type_t>(1, type_string));
  op_sub.overload(type_string, std::vector<type_t>(1, type_word));
  op_sub.overload(type_string, std::vector<type_t>(1, type_word_set));
  op_sub.addExplicitName("substraction operator");
  functions_.push_back(op_sub);

  FunctionDescriptor op_assign = FunctionDescriptor("opAssign", type_string, std::vector<type_t>(1, type_string));
  op_assign.overload(type_string, std::vector<type_t>(1, type_word));
  op_assign.overload(type_string, std::vector<type_t>(1, type_word_set));
  op_assign.addExplicitName("assignment operator");
  functions_.push_back(op_assign);
}

std::string StringType::add(std::string text, size_t line_start, size_t line_stop)
{
  std::string id = "__string[" + std::to_string(nb_strings_) + "]";
  nb_strings_++;

  StringBlock_t tmp;
  tmp.strings = text;
  tmp.lines_count.setStart(line_start);
  tmp.lines_count.setStop(line_stop);
  strings_[id] = tmp;

  return id;
}

std::string StringType::get(std::string id)
{
  if(strings_.find(id) != strings_.end())
    return strings_[id].strings;
  return "";
}

size_t StringType::nbLines(std::string id)
{
  if(strings_.find(id) != strings_.end())
    return strings_[id].lines_count.getNbLines();
  return 0;
}

size_t StringType::size(std::string id)
{
  if(strings_.find(id) != strings_.end())
    return strings_[id].strings.size();
  return 0;
}
