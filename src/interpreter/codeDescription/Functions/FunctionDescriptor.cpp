#include "ontoloGenius/interpreter/codeDescription/Functions/FunctionDescriptor.h"

FunctionDescriptor::FunctionDescriptor(std::string name, type_t return_type, std::vector<type_t> params_type)
{
  name_ = name;
  params_.push_back(params_type);
  return_.push_back(return_type);
}

bool FunctionDescriptor::overload(type_t return_type, std::vector<type_t> params_type)
{
  if(!testParams(params_type))
  {
    params_.push_back(params_type);
    return_.push_back(return_type);
    return true;
  }
  else
    return false;
}

std::string FunctionDescriptor::getName()
{
  return name_;
}

std::string FunctionDescriptor::getExplicitName()
{
  if(explicit_name_ == "")
    return name_;
  else
    return explicit_name_;
}

type_t FunctionDescriptor::getReturnType(std::vector<type_t> params)
{
  for(size_t i = 0; i < params_.size(); i++)
  {
    if(params_[i].size() == params.size())
    {
      bool find = true;
      for(size_t j = 0; j < params.size(); j++)
      {
        if(params[j] != params_[i][j])
          find = false;
      }

      if(find)
        return return_[i];
    }
  }

  return type_unknow;
}

bool FunctionDescriptor::testParams(std::vector<type_t> params)
{
  for(size_t i = 0; i < params_.size(); i++)
  {
    if(params_[i].size() == params.size())
    {
      bool find = true;
      for(size_t j = 0; j < params.size(); j++)
      {
        if(params[j] != params_[i][j])
          find = false;
      }

      if(find)
        return true;
    }
  }

  return false;
}

size_t FunctionDescriptor::testNbParams(size_t nb)
{
  size_t nb_param = 0;
  size_t dist = -1;

  for(size_t i = 0; i < params_.size(); i++)
  {
    if(params_[i].size() == nb)
    {
      dist = 0;
      nb_param = nb;
      break;
    }
    else
    {
      size_t tmp_dist;
      if(params_[i].size() > nb)
        tmp_dist = params_[i].size() - nb;
      else
        tmp_dist = nb - params_[i].size();

      if(tmp_dist < dist)
      {
        dist = tmp_dist;
        nb_param = params_[i].size();
      }
    }
  }

  return nb_param;
}

std::string FunctionDescriptor::getDeclaration(size_t nb)
{
  size_t index = 0;
  size_t dist = -1;

  for(size_t i = 0; i < params_.size(); i++)
  {
    if(params_[i].size() == nb)
    {
      dist = 0;
      index = i;
      break;
    }
    else
    {
      size_t tmp_dist;
      if(params_[i].size() > nb)
        tmp_dist = params_[i].size() - nb;
      else
        tmp_dist = nb - params_[i].size();

      if(tmp_dist < dist)
      {
        dist = tmp_dist;
        index = i;
      }
    }
  }

  std::string declaration = name_ + "(";
  for(size_t i = 0; i < params_[index].size(); i++)
  {
    declaration += to_string(params_[index][i]);
    if(i < params_[index].size() - 1)
      declaration += ", ";
  }
  declaration += ")";

  return declaration;
}

std::string FunctionDescriptor::to_string(type_t type)
{
  if(type == type_void)
    return "void";
  else if(type == type_string)
    return "string";
  else if(type == type_word)
    return "word";
  else if(type == type_word_set)
    return "word_set";
  else if(type == type_bool)
    return "bool";
  else if(type == type_unknow)
    return "unknow";
  else
    return "";
}
