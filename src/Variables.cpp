#include "ontoloGenius/Variables.h"

Variables::Variables() : Namespace("var")
{
  nb_variables_ = 0;
}

std::string Variables::add(std::string name)
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
    id = "__var(" + std::to_string(nb_variables_) + ")";
    nb_variables_++;

    Variable_t tmp;
    tmp.name = name;
    var_[id] = tmp;
  }

  return id;
}

std::string Variables::name(std::string id)
{
  if(var_.find(id) != var_.end())
    return var_[id].name;
  return "";
}

std::set<std::string> Variables::get(std::string id)
{
  if(var_.find(id) != var_.end())
    return var_[id].values;
  return std::set<std::string>();
}

std::string Variables::toString(std::string id)
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

bool Variables::set(std::string id, std::string value)
{
  remove(id);
  insert(id, value);
}

bool Variables::set(std::string id, std::set<std::string> value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values = value;
    return true;
  }
  return false;
}

bool Variables::insert(std::string id, std::string value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.insert(value);
    return true;
  }
  return false;
}

bool Variables::insert(std::string id, std::set<std::string> value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.insert(value.begin(), value.end());
    return true;
  }
  return false;
}

bool Variables::remove(std::string id)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.clear();
    return true;
  }
  return false;
}

bool Variables::remove(std::string id, std::string value)
{
  if(var_.find(id) != var_.end())
  {
    var_[id].values.erase(value);
    return true;
  }
  return false;
}

bool Variables::remove(std::string id, std::set<std::string> value)
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

size_t Variables::size(std::string id)
{
  if(var_.find(id) != var_.end())
    return var_[id].values.size();
  return 0;
}
