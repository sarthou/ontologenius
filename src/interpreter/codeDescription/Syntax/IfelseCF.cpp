#include "ontoloGenius/interpreter/codeDescription/Syntax/IfelseCF.h"

#include "ontoloGenius/interpreter/codeDescription/Code.h"
#include "ontoloGenius/interpreter/Error.h"

/*
* replace condition (if and else) by the key word __ifelse with the corresponding index
*/
void IfelseCF::compact(Code& code, Error* error)
{
  int nb_block = 0;
  size_t pose = 0;
  bool eof = false;

  do
  {
    pose = getNextIfBlock(nb_block, pose, code, error);
    if(pose == std::string::npos)
      eof = true;
  }
  while(!eof);

  if(error)
  {
    eof = false;
    pose = 0;
    do
    {
      size_t else_error = code.text.find("else", pose);
      if(else_error == std::string::npos)
        eof = true;
      else
      {
        size_t prev = else_error - 1;
        size_t post = else_error + 4;
        if(((code.text[prev] == ' ') || (code.text[prev] == '\n') || (code.text[prev] == ';') || (code.text[prev] == ')') || (code.text[prev] == '(')) &&
          ((code.text[post] == ' ') || (code.text[post] == '\n') || (code.text[post] == ';') || (code.text[post] == ')') || (code.text[post] == '(')))
          {
            size_t err_pose = getCorrectCharPosition(code.text, else_error);
            error->printError(err_pose, "‘else’ without a previous ‘if’");
            eof = true;
          }
          else
            pose = else_error + 1;
      }
    }
    while(!eof);
  }
}

size_t IfelseCF::getNextIfBlock(int& nb_block, size_t pose, Code& code, Error* error)
{
  size_t end = 0;

  size_t if_start = code.text.find("if", pose);
  if(if_start == std::string::npos)
    return std::string::npos;
  else
  {
    IfBlock_t if_block;
    pose = code.getInBraquet(if_start+2, if_block.IfBlock_condition, code.text);
    if_block.cond_pose = code.text.find("(", if_start) +1;

    if(pose == if_start+2)
    {
      if(error)
        error->printError(pose, "expected ‘(’ after 'if’");
      return std::string::npos;
    }
    else if(pose == std::string::npos)
    {
      if(error)
        error->printError(if_start+2, "expected corresponding ‘)’ after previous '(’");
      return std::string::npos;
    }

    if(code.findAfter(pose, "if") != std::string::npos)
      getNextIfBlock(nb_block, pose, code, error);

    size_t semicolon = code.text.find(";", pose);
    if_block.IfBlock_if = code.text.substr(pose+1, semicolon-pose);
    if_block.if_pose = pose+1;
    if_block.lines_count.setStart(code.getLineNumber(if_start));
    if_block.lines_count.setStop(code.getLineNumber(semicolon));
    pose = semicolon;
    end = semicolon;

    if(code.findBefore(if_start, '='))
      return code.text.find(";", if_start);

    size_t else_start = code.findAfter(pose, "else");
    if(else_start != std::string::npos)
    {
      if(code.findAfter(else_start+4, "if") != std::string::npos)
        getNextIfBlock(nb_block, else_start, code, error);

      semicolon = code.text.find(";", else_start);
      if_block.IfBlock_else = code.text.substr(else_start+4, semicolon-else_start-4+1);
      if_block.else_pose = else_start+4;
      if_block.lines_count.setStop(code.getLineNumber(semicolon));
      pose = semicolon;
      end = semicolon;
    }

    ifelse_code_["__ifelse[" + std::to_string(nb_block) + "];"] = code.text.substr(if_start, end-if_start+1);
    code.text.replace(if_start, end-if_start+1, "__ifelse[" + std::to_string(nb_block) + "];");

    ifelse_["__ifelse[" + std::to_string(nb_block) + "];"] = if_block;

    pose -= (end-if_start+1) - std::string("__ifelse[" + std::to_string(nb_block) + "];").size();
    nb_block++;
  }

  return pose;
}

bool IfelseCF::uncompact(Code& code)
{
  bool worked = false;
  while(code.text.find("__ifelse[") != std::string::npos)
  {
    size_t ifelse_pose = code.text.find("__ifelse[");
    size_t ifelse_end = code.text.find("]", ifelse_pose);
    std::string ifelse = code.text.substr(ifelse_pose, ifelse_end-ifelse_pose+2);
    std::string initial_code = code.ifelse_.ifelse_code_[ifelse];
    code.text.replace(ifelse_pose, ifelse.size(), initial_code);
    worked = true;
  }

  return worked;
}

size_t IfelseCF::getCorrectCharPosition(std::string text, size_t pose)
{
  bool end = false;
  while(!end)
  {
    size_t ifelse_pose = text.find("__ifelse[");
    if((ifelse_pose != std::string::npos) && (ifelse_pose < pose))
    {
      size_t ifelse_end = text.find("]", ifelse_pose);
      std::string ifelse = text.substr(ifelse_pose, ifelse_end-ifelse_pose+2);
      std::string initial_code = ifelse_code_[ifelse];
      text.replace(ifelse_pose, ifelse.size(), initial_code);
      if(ifelse_pose < pose)
        pose += initial_code.size() - ifelse.size();
    }
    else
      end = true;
  }
  return pose;
}
