#include "ontoloGenius/codeDescription/Functions/FunctionDescriptor.h"

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
