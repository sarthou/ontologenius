#include "ontoloGenius/codeDescription/Types/VariablesType.h"

VariablesType::VariablesType() : Namespace("var")
{
  nb_variables_ = 0;

  FunctionDescriptor to_string = FunctionDescriptor("toString", type_string, std::vector<type_t>(1, type_void));
  functions_.push_back(to_string);

  FunctionDescriptor empty = FunctionDescriptor("empty", type_bool, std::vector<type_t>(1, type_void));
  functions_.push_back(empty);

  FunctionDescriptor op_add = FunctionDescriptor("opAdd", type_word_set, std::vector<type_t>(1, type_word_set));
  op_add.overload(type_word_set, std::vector<type_t>(1, type_word));
  op_add.addExplicitName("addition operator");
  functions_.push_back(op_add);

  FunctionDescriptor op_sub = FunctionDescriptor("opSub", type_word_set, std::vector<type_t>(1, type_word_set));
  op_sub.overload(type_word_set, std::vector<type_t>(1, type_word));
  op_sub.addExplicitName("substraction operator");
  functions_.push_back(op_sub);

  FunctionDescriptor op_assign = FunctionDescriptor("opAssign", type_word_set, std::vector<type_t>(1, type_word_set));
  op_assign.overload(type_word_set, std::vector<type_t>(1, type_word));
  op_assign.addExplicitName("assignment operator");
  functions_.push_back(op_assign);

  FunctionDescriptor op_add_assign = FunctionDescriptor("opAddAssign", type_word_set, std::vector<type_t>(1, type_word_set));
  op_add_assign.overload(type_word_set, std::vector<type_t>(1, type_word));
  op_add_assign.addExplicitName("add and assigne operator");
  functions_.push_back(op_add_assign);

  FunctionDescriptor op_sub_assign = FunctionDescriptor("opSubAssign", type_word_set, std::vector<type_t>(1, type_word_set));
  op_sub_assign.overload(type_word_set, std::vector<type_t>(1, type_word));
  op_sub_assign.addExplicitName("substract and assigne operator");
  functions_.push_back(op_sub_assign);
}

std::string VariablesType::add(std::string name)
{
  std::string id = "";

  std::map<std::string, Variable_t>::iterator it = var_.begin();
  while(it != var_.end())
  {
    if((*it).second.name == name)
    {
      id = (*it).first;
      break;
    }
    it++;
  }

  if(id == "")
  {
    id = "__var[" + std::to_string(nb_variables_) + "]";
    nb_variables_++;

    Variable_t tmp;
    tmp.name = name;
    var_[id] = tmp;
  }

  return id;
}

std::string VariablesType::name(std::string id)
{
  if(var_.find(id) != var_.end())
    return var_[id].name;
  return "";
}

std::set<std::string> VariablesType::get(std::string id)
{
  if(var_.find(id) != var_.end())
    return var_[id].values;
  return std::set<std::string>();
}

std::string VariablesType::toString(std::string id)
{
  std::string str = "";
  if(var_.find(id) != var_.end())
  {
    std::set<std::string>::iterator it = var_[id].values.begin();
    while(it != var_[id].values.end())
    {
      str += " " + (*it);
      it++;
    }
    str += " ";
  }
  return str;
}

bool VariablesType::set(std::string id, std::string value)
{
  remove(id);
  insert(id, value);
  return true;
}

bool VariablesType::set(std::string id, std::set<std::string> value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values = value;
    return true;
  }
  return false;
}

bool VariablesType::insert(std::string id, std::string value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.insert(value);
    return true;
  }
  return false;
}

bool VariablesType::insert(std::string id, std::set<std::string> value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.insert(value.begin(), value.end());
    return true;
  }
  return false;
}

bool VariablesType::remove(std::string id)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.clear();
    return true;
  }
  return false;
}

bool VariablesType::remove(std::string id, std::string value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.erase(value);
    return true;
  }
  return false;
}

bool VariablesType::remove(std::string id, std::set<std::string> value)
{
  if(var_.find(id) != var_.end())
  {
    std::set<std::string>::iterator it = value.begin();
    while(it != value.end())
    {
      var_[id].values.erase(*it);
      it++;
    }
    return true;
  }
  return false;
}

size_t VariablesType::size(std::string id)
{
  if(var_.find(id) != var_.end())
    return var_[id].values.size();
  return 0;
}
