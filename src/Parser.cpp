#include <iostream>

#include "ontoloGenius/Parser.h"
#include "ontoloGenius/Computer.h"
#include "ontoloGenius/Compiler.h"

Parser::Parser(Code* code, size_t current_line, bool subparser) : code_(code), error_(code_)
{
  subparser_ = subparser;

  code_->lines_counter_.current_line_ = current_line;
  code_->lines_counter_.setStart(current_line);
  code_->first_char_pose = 0;

  code_->remove('\t');

  checkCode();
}

Parser::~Parser()
{
}

void Parser::checkCode()
{
  if(!subparser_)
  {
    checkReservedWord("__string");
    checkReservedWord("__comment");
    if(error_.isOnError())
      return;

    checkStringAndComment();

    checkReserved();
    if(error_.isOnError())
      return;

    checkBraquets();
  }

  getSubsections();
  if(error_.isOnError())
    return;

  getFromNamespace();

  replaceOperator();
  if(error_.isOnError())
    return;

  code_->ifelse_.compact(*code_, &error_);
  if(error_.isOnError())
    return;

  std::map<size_t, std::string> splited = splitBySemicolon();
  if(error_.isOnError())
    return;

  checkInstructionValidity(splited);

  if(error_.isOnError())
    return;

  Compiler compiler(code_);
  compiler.compile();
  error_.cpy(compiler.getError());
}

void Parser::checkReserved()
{
  checkReservedWord("__subsection");
  checkReservedWord("__ifelse");
  checkReservedWord("__ont");
  checkReservedWord("__var");
}

void Parser::checkReservedWord(std::string symbol)
{
  bool eof = false;
  size_t pose = 0;

  do
  {
    size_t find = code_->text.find(symbol, pose);
    if(find == std::string::npos)
      eof = true;
    else
    {
      error_.printError(find, std::string("The name ‘" + symbol + "’ is a reserved word of C^ language"));
      pose = find+1;
    }
  }
  while(!eof);
}

void Parser::checkStringAndComment()
{
  int16_t nb_comments = 0;

  for(size_t pose = 0; pose < code_->text.size(); pose++)
  {
    if((code_->text[pose] == '/') && (code_->text[pose+1] == '/'))
    {
      size_t new_line = code_->text.find("\n", pose);
      CommentBlock_t comment_i;
      comment_i.comment = code_->text.substr(pose, new_line-pose);
      comment_i.lines_count.setStart(code_->getLineNumber(pose));
      comment_i.lines_count.setStop(comment_i.lines_count.getStart());
      code_->comments_["__comment[" + std::to_string(nb_comments) + "]"] = comment_i;
      code_->text.replace(pose, new_line-pose, "__comment[" + std::to_string(nb_comments) + "]");

      nb_comments++;
    }
    else if((code_->text[pose] == '/') && (code_->text[pose+1] == '*'))
    {
      size_t comment_end = code_->text.find("*/", pose+2);
      if(comment_end != std::string::npos)
      {
        CommentBlock_t comment_i;
        comment_i.comment = code_->text.substr(pose, comment_end-pose+2);
        comment_i.lines_count.setStart(code_->getLineNumber(pose));
        comment_i.lines_count.setStop(code_->getLineNumber(comment_end));
        code_->comments_["__comment[" + std::to_string(nb_comments) + "]"] = comment_i;
        code_->text.replace(pose, comment_end-pose+2, "__comment[" + std::to_string(nb_comments) + "]");

        nb_comments++;
      }
      else
        error_.printError(pose, "expected ‘*/’ at end of input");
    }
    else if((code_->text[pose] == '*') && (code_->text[pose+1] == '/'))
    {
      error_.printError(pose, "expected primary-expression before ‘*/’ token");
    }
    else if(code_->text[pose] == '"')
    {
      size_t string_end = code_->text.find("\"", pose+1);
      if(string_end != std::string::npos)
      {
        std::string text = code_->text.substr(pose+1, string_end-pose - 1);
        std::string id = code_->strings_.add(text, code_->getLineNumber(pose), code_->getLineNumber(string_end));
        code_->text.replace(pose, string_end-pose+1, id);
      }
      else
        error_.printError(pose, "expected ‘\"’ at end of input");
    }
  }
}

void Parser::checkBraquets()
{
  int16_t nb_bracket = 0;
  size_t bracket_open, bracket_close = 0;

  for(size_t pose = 0; pose < code_->text.size(); pose++)
  {
    if(code_->text[pose] == '(')
    {
      nb_bracket++;
      if(nb_bracket == 1)
        bracket_open = pose;
    }
    else if(code_->text[pose] == ')')
    {
      if(nb_bracket == 0)
      {
        bracket_close = pose;
        error_.printError(bracket_close, "expected primary-expression before ‘)’ token");
      }
      else
        nb_bracket--;
    }
    else if(code_->text[pose] == ';')
    {
      if(nb_bracket > 0)
        error_.printError(bracket_open, "expected corresponding ‘)’ after previous '(’");

      nb_bracket = 0;
    }
  }
}

/*
* replace subsection {} by the key word __sebsection with the corresponding index
*/
void Parser::getSubsections()
{
  int nb_sub = 0;
  size_t begin = 0;
  bool eof = false;

  do
  {
    size_t bracket = code_->text.find("{", begin);
    if(bracket == std::string::npos)
      eof = true;
    else
    {
      size_t bracket = code_->text.find("{", begin);
      size_t first_bracket = bracket;
      int cpt = 1;
      while((cpt != 0) && (code_->text[bracket] != '\0'))
      {
        ++bracket;
        if(code_->text[bracket] == '{')
          cpt++;
        else if(code_->text[bracket] == '}')
          cpt--;

      }

      if(cpt == 0)
      {
        SubsectionBlock_t subsection_i;
        subsection_i.subsection = new Code(code_->text.substr(first_bracket+1, bracket-first_bracket-1));
        subsection_i.lines_count.setStart(code_->getLineNumber(first_bracket));
        subsection_i.lines_count.setStop(code_->getLineNumber(bracket));

        code_->subsections_["__subsection[" + std::to_string(nb_sub) + "];"] = subsection_i;
        code_->text.replace(first_bracket, bracket-first_bracket+1, "__subsection[" + std::to_string(nb_sub) + "];");

        /*send subsection to an other parser*/
        subsection_i.subsection->comments_ = code_->comments_;
        subsection_i.subsection->strings_ = code_->strings_;
        Parser parser(subsection_i.subsection, subsection_i.lines_count.getStart(), true);
        error_.cpy(parser.getError());

        nb_sub++;
      }
      else
      {
        error_.printError(first_bracket, "expected ‘}’ at end of input");
        eof = true;
      }
    }
  }
  while(!eof);

  eof = false;
  size_t bad_subsection = 0;
  do
  {
    bad_subsection = code_->text.find("}", bad_subsection);
    if(bad_subsection == std::string::npos)
      eof = true;
    else
    {
      error_.printError(bad_subsection, "expected primary-expression before ‘}’ token");
      bad_subsection++;
    }
  }
  while(!eof);
}

void Parser::getFromNamespace()
{
  bool eof = false;

  do
  {
    size_t ns_pose = code_->text.find(":", 0);
    if(ns_pose == std::string::npos)
      eof = true;
    else
    {
      if(code_->findHere(ns_pose+1, ":"))
      {
        std::string ns = code_->getWordBefore(ns_pose);
        if(code_->variables_.isThisNamespace(ns))
        {
          std::string var = code_->getWordAfter(ns_pose+1);
          std::string var_id = code_->variables_.add(var);
          code_->text.replace(ns_pose - 3, var.size() + 5, var_id);
        }
        else if(ns == "ont")
          code_->text.replace(ns_pose - 3, 5, "__ont.");
        else
        {
          error_.printError(ns_pose - ns.size(), "unknow namespace '" + ns + "'");
          eof = true;
        }
      }
      else
      {
        error_.printError(ns_pose, "unknow operator ':'");
        eof = true;
      }
    }
  }
  while(!eof);
}

void Parser::replaceOperator()
{
  code_->operators_.describe("+=", "opAddAssign", true, 0);
  code_->operators_.describe("-=", "opSubAssign", true, 0);
  code_->operators_.describe("-", "opSub", false, 0);
  code_->operators_.describe("+", "opAdd", false, 0);
  code_->operators_.describe("=", "opAssign", true, 0);
  //code_->operators_.describe("=if", "opLoop", true, 0);
  code_->operators_.dontCarre("==");
  code_->operators_.dontCarre("!=");
  code_->operators_.dontCarre("=if");

  code_->operators_.op2Function();
}

std::map<size_t, std::string> Parser::splitBySemicolon()
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
      size_t if_pose = subcode.find("__ifelse[");
      if(if_pose == 0)
      {
        std::string ifelse_id = subcode + ";";
        offset += splitIfBlock(splited, ifelse_id);
      }
      else if(if_pose != std::string::npos)
        error_.printError(start+if_pose+offset, "expected ‘;’ before 'if'");
      else
          splited[start+offset] = subcode;

      start = stop +1;
    }
  }

  return splited;
}

int Parser::splitIfBlock(std::map<size_t, std::string>& splited, std::string ifelse_id)
{
  int offset = 0;

  /*Get code of if condition*/
  std::string if_code = code_->ifelse_[ifelse_id].IfBlock_if;
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
    else if(if_pose != std::string::npos)
      error_.printError(if_start+if_pose, "expected ‘;’ before 'if'");
    else
      splited[if_start] = if_code.substr(0, if_code.size() - 1);
  }

  /*Get code of else condition*/
  std::string else_code = code_->ifelse_[ifelse_id].IfBlock_else;
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
    else if(if_pose != std::string::npos)
      error_.printError(else_start+if_pose, "expected ‘;’ before 'if'");
    else
      splited[else_start] = else_code.substr(0, else_code.size() - 1);;
  }

  /*Check semicolon in condition and empty conditions*/
  std::string condition = code_->ifelse_[ifelse_id].IfBlock_condition;
  size_t condition_start = code_->ifelse_[ifelse_id].cond_pose;
  code_->goToEffectiveCode(condition, condition_start);

  if(condition != "")
  {
    size_t semicolon = condition.find(";");
    if(semicolon != std::string::npos)
      error_.printError(condition_start + semicolon, "expected ‘)’ before ';'");
  }
  else
    error_.printError(condition_start, "expected primary-expression before ‘)’ token");

  return offset;
}

void Parser::checkInstructionValidity(std::map<size_t, std::string>& splited)
{
  for (std::map<size_t,std::string>::iterator it=splited.begin(); it!=splited.end(); ++it)
  {
    std::string code = it->second;
    size_t pose = it->first;
    code_->goToEffectiveCode(code, pose);
    size_t sub = code.find("__subsection[");
    if((sub == std::string::npos) || (sub != 0))
    {
      if(code.find(".") == std::string::npos)
        error_.printWarning(pose, "instruction with no effect");
      else
        checkInstructionValidity(pose, code);
    }

    if(error_.isOnError())
      break;
  }
}

void Parser::checkInstructionValidity(size_t pose, std::string code, bool onFunction)
{
  std::string on = "";
  std::string func = "";
  std::string arg = "";
  std::vector<std::string> args;
  std::vector<size_t> args_pose;
  TextManipulator manipulator(code);
  size_t word_integrity = 0;

  /*Getting founction object*/
  size_t dot = 0;
  if(onFunction == false)
  {
    dot = code.find(".", 0);
    on = code.substr(0, dot);
  }
  else
    on = "__onfunc";

  /*Getting function name*/
  size_t open_arg = code.find("(", dot + 1);
  func = code.substr(dot + 1, open_arg - dot - 1);
  word_integrity = code_->checkWordIntegrity(func);
  if(word_integrity != std::string::npos)
  {
    code_->removeProtectedWord(func);
    error_.printError(pose + dot + 1 + word_integrity, "invalid syntax after ‘" + func + "’");
  }

  if(open_arg != std::string::npos)
  {
    /*Getting arguments*/
    manipulator.getInBraquet(open_arg, arg, code);
    size_t close_arg = open_arg + arg.size() + 1;

    size_t start_arg = 0;
    while(arg.find(",", start_arg) != std::string::npos)
    {
      size_t nex_arg = arg.find(",", start_arg);
      args.push_back(arg.substr(start_arg, nex_arg - start_arg));
      args_pose.push_back(pose+open_arg+1+start_arg);
      start_arg = nex_arg+1;
    }
    args.push_back(arg.substr(start_arg));
    args_pose.push_back(pose+open_arg+1+start_arg);

    /*Checking arguments validity*/
    for(size_t i = 0; i < args.size(); i++)
      checkArgumentValidity(args_pose[i], args[i]);

    /*Checking if a function is applied on the function result*/
    size_t next_func = manipulator.findAfter(close_arg, ".");
    if(next_func != std::string::npos)
      checkInstructionValidity(pose + next_func+1, code.substr(next_func), true);
    else
    {
      /*Checking if no residual word is present after the end on instruction*/
      std::string next = manipulator.getWordAfter(close_arg, false);
      if(next != "")
      {
        size_t err_pose = code.find(next, close_arg);
        err_pose += pose;
        code_->removeProtectedWord(next);
        error_.printError(err_pose, "expected ';' before ‘" + next + "’");
      }
    }
  }
  else
    error_.printError(pose + dot + func.size(), "miss ‘(‘ after ‘" + func + "’");
}

void Parser::checkArgumentValidity(size_t pose, std::string code)
{
  code_->goToEffectiveCode(code, pose);
  if(code.find("__subsection[") == std::string::npos)
  {
    if(code.find(".") != std::string::npos)
      checkInstructionValidity(pose, code);
  }
  else
  {
    size_t sub = code.find("__subsection[");
    error_.printError(pose+sub, "unexpected ‘{’");
  }
}
