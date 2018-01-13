#include "ontoloGenius/codeDescription/Operators.h"

#include <iostream>

bool Operators::describe(std::string op, std::string function, bool whole_line, uint8_t priority)
{
  OperatorDescriptor_t tmp;
  tmp.op = op;
  tmp.function = function;
  tmp.priority = priority;
  tmp.whole_line = whole_line;
  tmp.dont_carre = false;
  descriptors_.push_back(tmp);
}

void Operators::dontCarre(std::string op)
{
  OperatorDescriptor_t tmp;
  tmp.op = op;
  tmp.dont_carre = true;
  descriptors_.push_back(tmp);
}

void Operators::op2Function()
{
  OperatorDescriptor_t* op = nullptr;
  for(size_t i = 0; i < code_->size(); i++)
  {
    if((op = isPreOperator(i)) != nullptr)
    {
      Operator_t tmp_op;
      tmp_op.op = op->op;
      tmp_op.begin = i - op->op.size() + 1;

      if(op->whole_line)
        tmp_op.end_braquet = code_->find(";", i);
      else
        tmp_op.end_braquet = findNextOperator(i+1);

      tmp_op.replace = "." + op->function + "(";
      code_->insert(tmp_op.end_braquet, ")");
      code_->replace(tmp_op.begin, tmp_op.op.size(), tmp_op.replace);

      operators_.push_back(tmp_op);
    }
  }
}

OperatorDescriptor_t* Operators::isPreOperator(size_t& pose)
{
  std::vector<OperatorDescriptor_t*> goods;
  for(size_t op = 0; op < descriptors_.size(); op++)
  {
    bool ok = true;

    for(size_t i = 0; i < descriptors_[op].op.size(); i++)
    {
      if(descriptors_[op].op[i] != (*code_)[pose + i])
      {
        ok = false;
        break;
      }
    }

    if(ok)
      goods.push_back(&descriptors_[op]);
  }

  if(goods.size() != 0)
  {
    OperatorDescriptor_t* op = nullptr;
    size_t max_size = 0;
    for(size_t i = 0; i < goods.size(); i++)
    {
      if(goods[i]->op.size() > max_size)
      {
        op = goods[i];
        max_size = goods[i]->op.size();
      }
    }

    pose += op->op.size() - 1;
    if(op->dont_carre == true)
      return nullptr;
    else
      return op;
  }
  else
    return nullptr;
}

OperatorDescriptor_t* Operators::isPostOperator(size_t pose)
{
  std::vector<OperatorDescriptor_t*> goods;
  size_t offset = 0;

  if((*code_)[pose] != '.')
    return nullptr;

    offset++;
    while(((*code_)[pose+offset] == ' ') || (*code_)[pose+offset] == '\n')
      offset++;

  for(size_t op = 0; op < descriptors_.size(); op++)
  {
    if(descriptors_[op].dont_carre == false)
    {
      bool ok = true;

      for(size_t i = 0; i < descriptors_[op].function.size(); i++)
      {
        if(descriptors_[op].function[i] != (*code_)[pose + offset + i])
        {
          ok = false;
          break;
        }
      }

      if(ok)
        goods.push_back(&descriptors_[op]);
    }
  }

  if(goods.size() != 0)
  {
    OperatorDescriptor_t* op = nullptr;
    size_t max_size = 0;
    for(size_t i = 0; i < goods.size(); i++)
    {
      if(goods[i]->function.size() > max_size)
      {
        op = goods[i];
        max_size = goods[i]->function.size();
      }
    }

    return op;
  }
  else
    return nullptr;
}

size_t Operators::findNextOperator(size_t pose) //TODO : add error
{
  bool find = false;
  OperatorDescriptor_t* op = nullptr;
  int16_t nb_braquet = 0;

  do
  {
    if((*code_)[pose] == '(')
    {
      nb_braquet++;
      pose++;
    }
    else if((*code_)[pose] == ')')
    {
      nb_braquet--;
      pose++;
    }
    else if((*code_)[pose] == ';')
      find = true;
    else if((op = isPreOperator(pose)) != nullptr)
    {
      if(nb_braquet == 0)
      {
        find = true;
        pose -= op->op.size() - 1;
      }
      else
        pose++;
    }
    else if((op = isPostOperator(pose)) != nullptr)
    {
      if(nb_braquet == 0)
        find = true;
      else
        pose++;
    }
    else
      pose++;
  }
  while(find == false);

  return pose;
}
