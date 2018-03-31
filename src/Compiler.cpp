#include "ontoloGenius/Compiler.h"

#include "ontoloGenius/codeDescription/Types/VariablesType.h"
#include "ontoloGenius/codeDescription/Functions/OntoFunctions.h"

#include "ontoloGenius/codeDescription/TextManipulator.h"

#include <iostream>

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE  "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN   "\x1B[1;92m"
#endif

size_t Compiler::section_cpt_ = 0;

Compiler::Compiler(Code* code) : code_(code), error_(code_)
{
}

size_t Compiler::compile()
{
  size_t section = section_cpt_;
  std::cout << "-------- section " << section << std::endl;
  section_cpt_++;

  std::map<size_t, std::string> splited = splitBySemicolon();
  compileIntructions(splited);

  std::cout << "-------- end of section " << section << std::endl;

  return section;
}

void Compiler::compileIntructions(std::map<size_t, std::string> splited)
{
  std::map<size_t, std::string>::iterator it = splited.begin();
  for(; it != splited.end(); ++it)
  {
    std::cout << COLOR_GREEN << "==== Compile instruction" << COLOR_OFF << std::endl;
    size_t dot = it->second.find(".");
    if(dot != std::string::npos)
    {
      std::cout << COLOR_ORANGE << "--- function" << COLOR_OFF << std::endl;
      std::string on = it->second.substr(0, dot);
      code_->removeNonEffectiveCode(on);
      std::string instruction = it->second.substr(dot + 1);

      if(on.find(" ") != std::string::npos)
        error_.printError(it->first+it->second.find(" "), "unexpected expression after '" + on.substr(0,it->second.find(" ")) + "'");
      else if(on.find("__var[") != std::string::npos)
        onVariableInstruction(on, instruction, it->first + dot + 1);
      else if(on.find("__ont") != std::string::npos)
        onOntologyInstruction(instruction, it->first + dot + 1);
      else
        error_.printWarning(it->first, "instruction with no effect");
    }
    else
    {
      std::cout << COLOR_ORANGE << "--- other" << COLOR_OFF << std::endl;
      if(it->second.find("__ifelse[") != std::string::npos)
        std::cout << "if condition" << std::endl;
      else if(it->second.find("__subsection[") != std::string::npos)
        std::cout << "subsection" << std::endl;
      else if(it->second.find("__var[") != std::string::npos)
        error_.printWarning(it->first, "instruction with no effect");
      else if(it->second.find("__ont") != std::string::npos)
        error_.printError(it->first+it->second.find("__ont")+5, "expected function after 'ont::'");
      else
        std::cout << "????????" << std::endl;
    }
  }
}

type_t Compiler::compileIntruction(std::string instruction, size_t pose)
{
  size_t dot = instruction.find(".");
  std::cout << COLOR_GREEN << "==compile sub instruction " << COLOR_OFF << instruction << std::endl;
  if(dot != std::string::npos)
  {
    std::cout << COLOR_ORANGE << "--funtion" << COLOR_OFF << std::endl;
    std::string on = instruction.substr(0, dot);
    code_->removeNonEffectiveCode(on);
    std::string subinstruction = instruction.substr(dot + 1);

    if(on.find(" ") != std::string::npos)
      error_.printError(pose+instruction.find(" "), "unexpected expression after '" + on.substr(0,instruction.find(" ")) + "'");
    if(instruction.find("__var[") != std::string::npos)
      return onVariableInstruction(on, subinstruction, pose + dot + 1);
    else if(instruction.find("__ont") != std::string::npos)
      return onOntologyInstruction(subinstruction, pose + instruction.find("__ont")+6);
    else
      return type_word_set; //TODO: create onPropertyInstruction
  }
  else
  {
    std::cout << COLOR_ORANGE << "--other" << COLOR_OFF << std::endl;
    if(instruction.find("__var[") != std::string::npos)
      return type_word_set;
    if(instruction.find("__string[") != std::string::npos)
      return type_string;
    else if(instruction.find("__ont") != std::string::npos)
      error_.printError(pose+instruction.find("__ont")+5, "expected function after 'ont::'");
    else if(instruction.find("__ifelse[") != std::string::npos)
      error_.printError(pose, "unexpected instruction");
    else if(instruction.find("__subsection[") != std::string::npos)
      error_.printError(pose, "unexpected instruction");
    else if(instruction == "")
      return type_void;
    else
      return type_word;
  }
  return type_unknow;
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

type_t Compiler::onVariableInstruction(std::string variable, std::string instruction, size_t pose)
{
  TextManipulator manipulator(instruction);
  size_t bracket = instruction.find("(");
  std::string function = instruction.substr(0,bracket);
  std::cout << "on variable " << variable << "  " << function <<  std::endl;
  VariablesType var;
  if(var.functionExist(function))
  {
    FunctionDescriptor* descriptor = var.findFunction(function);
    std::cout << "function " + descriptor->getExplicitName() + " exist" << std::endl;

    std::string arg;
    size_t bracket_end = manipulator.getInBraquet(bracket, arg, instruction);

    std::vector<type_t> args_types = compileParameters(arg, bracket+1, descriptor);
    if(descriptor->testParams(args_types))
      std::cout << "==> good args" << std::endl;
    else
      noMatchigFunction(pose + bracket, descriptor, args_types);

    /*if(args.size() != descriptor->testNbParams(args.size()))
      error_.printError(pose + bracket_end, "no matching function for call to"); //TODO complete message
      /*candidate: void Error::printError(size_t, std::__cxx11::string)
   void printError(size_t pose, std::string message);
*/
    //return descriptor->getReturnType()
  }
  else
    return type_word_set;

  return type_unknow;
}

type_t Compiler::onOntologyInstruction(std::string instruction, size_t pose)
{
  TextManipulator manipulator(instruction);
  size_t bracket = instruction.find("(");
  std::string function = instruction.substr(0,bracket);
  std::cout << "on onto " << function <<  std::endl;
  OntoFunctions onto;
  if(onto.functionExist(function) == false)
  {
    error_.printError(pose, "'" + function + "' is not a member of 'ont'");
    return type_unknow;
  }
  FunctionDescriptor* descriptor = onto.findFunction(function);
  std::cout << "function " + descriptor->getExplicitName() + " exist" << std::endl;

  std::string arg;
  size_t bracket_end = manipulator.getInBraquet(bracket, arg, instruction);

  std::vector<type_t> args_types = compileParameters(arg, bracket+1, descriptor);

  if(descriptor->testParams(args_types))
    std::cout << "==> good args" << std::endl;
  else
    std::cout << "==> bad args" << std::endl;

  //return descriptor->getReturnType()
  return type_unknow;
}

void Compiler::getParameters(std::string arg, size_t pose, std::vector<std::string>& args, std::vector<size_t>& args_pose)
{
  size_t start_arg = 0;
  while(arg.find(",", start_arg) != std::string::npos)
  {
    size_t nex_arg = arg.find(",", start_arg);
    args.push_back(arg.substr(start_arg, nex_arg - start_arg));
    args_pose.push_back(pose+1+start_arg);
    start_arg = nex_arg+1;
  }
  args.push_back(arg.substr(start_arg));
  args_pose.push_back(pose+1+start_arg);

  for(size_t i = 0; i < args.size(); i++)
  {
    code_->goToEffectiveCode(args[i], args_pose[i]);
    code_->removeNonEffectiveCode(args[i]);
  }
}

std::vector<type_t> Compiler::compileParameters(std::string arg, size_t pose, FunctionDescriptor* descriptor)
{
  std::vector<std::string> args;
  std::vector<size_t> args_pose;
  getParameters(arg, pose, args, args_pose);
  std::vector<type_t> args_types;

  if(args.size() > 0)
    for(size_t i = 0; i < args.size(); i++)
      args_types.push_back(compileIntruction(args[i], args_pose[i]));
  else
    args_types.push_back(type_void);

  for(size_t i = 0; i < args_types.size(); i++)
    std::cout << "arg " << i << " : " << args[i] << " of type : " << descriptor->to_string(args_types[i]) << std::endl;

  return args_types;
}

void Compiler::noMatchigFunction(size_t pose, FunctionDescriptor* descriptor, std::vector<type_t> args_types)
{
  std::string proto = descriptor->getName() + "(";
  for(size_t i = 0; i < args_types.size(); i++)
  {
    proto += descriptor->to_string(args_types[i]);
    if(i < args_types.size() - 1)
      proto += ", ";
  }
  proto += ")";

  error_.printError(pose, "no matching function for call to " + proto);

  std::vector<std::vector<type_t>> params_types = descriptor->getParameters();
  for(size_t i = 0; i < params_types.size(); i++)
  {
    std::string possible = descriptor->getName() + "(";
    for(size_t arg_i = 0; arg_i < params_types[i].size(); arg_i++)
    {
      possible += descriptor->to_string(params_types[i][arg_i]);
      if(arg_i < params_types[i].size() - 1)
        possible += ", ";
    }
    possible += ")";

    error_.printNote(pose, "candidate: " + possible);
  }
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
