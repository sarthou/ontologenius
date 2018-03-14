#include "ontoloGenius/Compiler.h"

#include "ontoloGenius/codeDescription/Types/VariablesType.h"
#include "ontoloGenius/codeDescription/Functions/OntoFunctions.h"

#include <iostream>

size_t Compiler::section_cpt_ = 0;

Compiler::Compiler(Code* code) : code_(code), error_(code_)
{
}

size_t Compiler::compile()
{
  size_t section = section_cpt_;
  std::cout << "section " << section << std::endl;
  section_cpt_++;

  std::map<size_t, std::string> splited = splitBySemicolon();
  compileIntructions(splited);

  return section;
}

void Compiler::compileIntructions(std::map<size_t, std::string> splited)
{
  std::map<size_t, std::string>::iterator it = splited.begin();
  for(; it != splited.end(); ++it)
  {
    std::cout << it->first << " : " << it->second << " : ";
    size_t dot = it->second.find(".");
    if(dot != std::string::npos)
    {
      std::string on = it->second.substr(0, dot);
      code_->removeNonEffectiveCode(on);
      std::string instruction = it->second.substr(dot + 1);

      if(on.find(" ") != std::string::npos)
        error_.printError(it->first+it->second.find(" "), "unexpected expression after '" + on.substr(0,it->second.find(" ")) + "'");
      if(it->second.find("__var[") != std::string::npos)
        onVariableInstruction(on, instruction);
      else if(it->second.find("__ont") != std::string::npos)
        onOntologyInstruction(instruction, it->first + it->second.find("__ont")+6);
      else
        error_.printWarning(it->first, "instruction with no effect");
    }
    else
    {
      if(it->second.find("__ifelse[") != std::string::npos)
        std::cout << "if condition" << std::endl;
      else if(it->second.find("__subsection[") != std::string::npos)
        std::cout << "subsection" << std::endl;
      else
        std::cout << "????????" << std::endl;
    }
    std::cout << std::endl;
  }
}

std::map<size_t, std::string> Compiler::splitBySemicolon()
{
  size_t start = 0;
  size_t stop = 0;
  std::map<size_t, std::string> splited;
  size_t offset = 0;

  while(stop != std::string::npos)
  {
    stop = code_->text.find(";", start);
    if(stop != std::string::npos)
    {
      std::string subcode = code_->text.substr(start, stop - start);
      code_->goToEffectiveCode(subcode, start);
      splited[start+offset] = subcode;

      size_t if_pose = subcode.find("__ifelse[");
      if(if_pose == 0)
        offset += getIfOffset(subcode + ";");

      start = stop +1;
    }
  }

  return splited;
}

int Compiler::getIfOffset(std::string ifelse_id)
{
  int offset = 0;

  /*Get offset of if condition*/
  std::string if_code = code_->ifelse_[ifelse_id].IfBlock_if;
  size_t if_start = code_->ifelse_[ifelse_id].if_pose;

  offset = code_->ifelse_.ifelse_code_[ifelse_id].size() - ifelse_id.size();

  code_->goToEffectiveCode(if_code, if_start);
  if(if_code != "")
  {
    size_t if_pose = if_code.find("__ifelse[");
    if(if_pose == 0)
      offset += getIfOffset(if_code);
  }

  /*Get offset of else condition*/
  std::string else_code = code_->ifelse_[ifelse_id].IfBlock_else;
  size_t else_start = code_->ifelse_[ifelse_id].else_pose;

  code_->goToEffectiveCode(else_code, else_start);
  if(else_code != "")
  {
    size_t if_pose = else_code.find("__ifelse[");
    if(if_pose == 0)
      offset += getIfOffset(else_code);
  }

  return offset;
}

void Compiler::onVariableInstruction(std::string variable, std::string instruction)
{
  size_t bracket = instruction.find("(");
  std::string function = instruction.substr(0,bracket);
  std::cout << "on variable " << variable << "  " << function <<  std::endl;
  VariablesType var;
  if(var.functionExist(function))
    std::cout << "function exist" << std::endl;
  else
    std::cout << "function don't exist" << std::endl;
}

void Compiler::onOntologyInstruction(std::string instruction, size_t pose)
{
  size_t bracket = instruction.find("(");
  std::string function = instruction.substr(0,bracket);
  std::cout << "on onto " << function <<  std::endl;
  OntoFunctions onto;
  if(onto.functionExist(function) == false)
  {
    error_.printError(pose, "'" + function + "' is not a member of 'ont'");
    return;
  }
  std::cout << "function exist" << std::endl;
}

/*int Compiler::splitIfBlock(std::map<size_t, std::string>& splited, std::string ifelse_id)
{
  int offset = 0;

  /*Get code of if condition*/
  /*std::string if_code = code_->ifelse_[ifelse_id].IfBlock_if;
  size_t if_start = code_->ifelse_[ifelse_id].if_pose;

  offset = code_->ifelse_.ifelse_code_[ifelse_id].size() - ifelse_id.size();

  code_->goToEffectiveCode(if_code, if_start);
  if(if_code != "")
  {
    size_t if_pose = if_code.find("__ifelse[");
    if(if_pose == 0)
    {
      std::string sub_ifelse_id = if_code;
      offset += splitIfBlock(splited, sub_ifelse_id);
    }
    else
      splited[if_start] = if_code.substr(0, if_code.size() - 1);
  }*/

  /*Get code of else condition*/
  /*std::string else_code = code_->ifelse_[ifelse_id].IfBlock_else;
  size_t else_start = code_->ifelse_[ifelse_id].else_pose;

  code_->goToEffectiveCode(else_code, else_start);
  if(else_code != "")
  {
    size_t if_pose = else_code.find("__ifelse[");
    if(if_pose == 0)
    {
      std::string sub_ifelse_id = else_code;
      offset += splitIfBlock(splited, sub_ifelse_id);
    }
    else
      splited[else_start] = else_code.substr(0, else_code.size() - 1);;
  }

  return offset;
}*/
