#include <iostream>

#include "ontoloGenius/Parser.h"
#include "ontoloGenius/Computer.h"

Parser::Parser(std::string code, TreeObject& onto, size_t current_line) : onto_(onto), code_(code), error_(&code_)
{
  subparser_ = nullptr;
  begin_ = end_ = 0;

  code_.lines_counter_.current_line_ = current_line;
  code_.lines_counter_.setStart(current_line);

  code_.print();
  std::cout << "----------------------" << std::endl;

  code_.remove('\t');

  checkReservedWord("__string");
  checkReservedWord("__comment");

  checkStringAndComment();

  checkReserved();

  checkBraquets();

  getSubsections();

  getFromNamespace();

  replaceOperator();

  getIfBlock();

  code_.print();
  std::cout << "----------------------" << std::endl;

  error_.printStatus();
}

Parser::~Parser()
{
  if(subparser_ != nullptr)
    delete subparser_;
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
    size_t find = code_.text.find(symbol, pose);
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

  for(size_t pose = 0; pose < code_.text.size(); pose++)
  {
    if((code_.text[pose] == '/') && (code_.text[pose+1] == '/'))
    {
      size_t new_line = code_.text.find("\n", pose);
      CommentBlock_t comment_i;
      comment_i.comment = code_.text.substr(pose, new_line-pose);
      comment_i.lines_count.setStart(code_.getLineNumber(pose));
      comment_i.lines_count.setStop(comment_i.lines_count.getStart());
      code_.comments_["__comment(" + std::to_string(nb_comments) + ")"] = comment_i;
      code_.text.replace(pose, new_line-pose, "__comment(" + std::to_string(nb_comments) + ")");

      nb_comments++;
    }
    else if((code_.text[pose] == '/') && (code_.text[pose+1] == '*'))
    {
      size_t comment_end = code_.text.find("*/", pose+2);
      if(comment_end != std::string::npos)
      {
        CommentBlock_t comment_i;
        comment_i.comment = code_.text.substr(pose, comment_end-pose+2);
        comment_i.lines_count.setStart(code_.getLineNumber(pose));
        comment_i.lines_count.setStop(code_.getLineNumber(comment_end));
        code_.comments_["__comment(" + std::to_string(nb_comments) + ")"] = comment_i;
        code_.text.replace(pose, comment_end-pose+2, "__comment(" + std::to_string(nb_comments) + ")");

        nb_comments++;
      }
      else
        error_.printError(pose, "expected ‘*/’ at end of input");
    }
    else if((code_.text[pose] == '*') && (code_.text[pose+1] == '/'))
    {
      error_.printError(pose, "expected primary-expression before ‘*/’ token");
    }
    else if(code_.text[pose] == '"')
    {
      size_t string_end = code_.text.find("\"", pose+1);
      if(string_end != std::string::npos)
      {
        std::string text = code_.text.substr(pose+1, string_end-pose - 1);
        std::string id = code_.strings_.add(text, code_.getLineNumber(pose), code_.getLineNumber(string_end));
        code_.text.replace(pose, string_end-pose+1, id);
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

  for(size_t pose = 0; pose < code_.text.size(); pose++)
  {
    if(code_.text[pose] == '(')
    {
      nb_bracket++;
      if(nb_bracket == 1)
        bracket_open = pose;
    }
    else if(code_.text[pose] == ')')
    {
      nb_bracket--;
      if(nb_bracket == -1)
        bracket_close = pose;
    }
    else if(code_.text[pose] == ';')
    {
      if(nb_bracket > 0)
        error_.printError(bracket_open, "expected corresponding ‘)’ after previous '(’");
      else if(nb_bracket < 0)
        error_.printError(bracket_close, "expected primary-expression before ‘(’ token");

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
    size_t bracket = code_.text.find("{", begin);
    if(bracket == std::string::npos)
      eof = true;
    else
    {
      size_t bracket = code_.text.find("{", begin);
      size_t first_bracket = bracket;
      int cpt = 1;
      while((cpt != 0) && (code_.text[bracket] != '\0'))
      {
        ++bracket;
        if(code_.text[bracket] == '{')
          cpt++;
        else if(code_.text[bracket] == '}')
          cpt--;

      }

      if(cpt == 0)
      {
        SubsectionBlock_t subsection_i;
        subsection_i.subsection = code_.text.substr(first_bracket+1, bracket-first_bracket-1);
        subsection_i.lines_count.setStart(code_.getLineNumber(first_bracket));
        subsection_i.lines_count.setStop(code_.getLineNumber(bracket));

        code_.subsections_["__subsection(" + std::to_string(nb_sub) + ");"] = subsection_i;
        code_.text.replace(first_bracket, bracket-first_bracket+1, "__subsection(" + std::to_string(nb_sub) + ");");

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
    bad_subsection = code_.text.find("}", bad_subsection);
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

/*
* replace condition (if and else) by the key word __ifBlock with the corresponding index
*/
void Parser::getIfBlock()
{
  int nb_block = 0;
  size_t pose = 0;
  bool eof = false;

  do
  {
    pose = getNextIfBlock(nb_block, pose);
    if(pose == std::string::npos)
      eof = true;
  }
  while(!eof);

  eof = false;
  pose = 0;
  do
  {
    size_t else_error = code_.text.find("else", pose);
    if(else_error == std::string::npos)
      eof = true;
    else
    {
      if(((code_.text[else_error - 1] == ' ') || (code_.text[else_error - 1] == '\n') || (code_.text[else_error - 1] == ';')) &&
        ((code_.text[else_error + 4] == ' ') || (code_.text[else_error + 4 ] == '\n') || (code_.text[else_error + 4] == ';')))
        {
          error_.printError(else_error, "‘else’ without a previous ‘if’");
          eof = true;
        }
        else
          pose = else_error + 1;
    }
  }
  while(!eof);
}

size_t Parser::getNextIfBlock(int& nb_block, size_t pose)
{
  size_t end = 0;

  size_t if_start = code_.text.find("if", pose);
  if(if_start == std::string::npos)
    return std::string::npos;
  else
  {
    IfBlock_t if_block;
    pose = code_.getInBraquet(if_start+2, if_block.IfBlock_condition, code_.text);

    if(pose == if_start+2)
    {
      error_.printError(pose, "expected ‘(’ after 'if’");
      return std::string::npos;
    }
    else if(pose == std::string::npos)
    {
      error_.printError(if_start+2, "expected corresponding ‘)’ after previous '(’");
      return std::string::npos;
    }

    if(code_.findAfter(pose, "if") != std::string::npos)
      getNextIfBlock(nb_block, pose);

    size_t semicolon = code_.text.find(";", pose);
    if_block.IfBlock_if = code_.text.substr(pose+1, semicolon-pose);
    if_block.lines_count.setStart(code_.getLineNumber(if_start));
    if_block.lines_count.setStop(code_.getLineNumber(semicolon));
    pose = semicolon;
    end = semicolon;

    if(code_.findBefore(if_start, '='))
      return code_.text.find(";", if_start);

    size_t else_start = code_.findAfter(pose, "else");
    if(else_start != std::string::npos)
    {
      if(code_.findAfter(else_start+4, "if") != std::string::npos)
        getNextIfBlock(nb_block, else_start);

      semicolon = code_.text.find(";", else_start);
      if_block.IfBlock_else = code_.text.substr(else_start+4, semicolon-else_start-4+1);
      if_block.lines_count.setStop(code_.getLineNumber(semicolon));
      pose = semicolon;
      end = semicolon;
    }

    code_.ifelse_["__ifelse(" + std::to_string(nb_block) + ");"] = if_block;
    code_.text.replace(if_start, end-if_start+1, "__ifelse(" + std::to_string(nb_block) + ");");

    pose -= (end-if_start+1) - std::string("__ifelse(" + std::to_string(nb_block) + ");").size();

    nb_block++;
  }

  return pose;
}

void Parser::getFromNamespace()
{
  bool eof = false;

  do
  {
    size_t ns_pose = code_.text.find(":", 0);
    if(ns_pose == std::string::npos)
      eof = true;
    else
    {
      if(code_.findHere(ns_pose+1, ":"))
      {
        std::string ns = code_.getWordBefore(ns_pose);
        if(code_.variables_.isThisNamespace(ns))
        {
          std::string var = code_.getWordAfter(ns_pose+1);
          std::string var_id = code_.variables_.add(var);
          code_.text.replace(ns_pose - 3, var.size() + 5, var_id);
        }
        else if(ns == "ont") //TODO : create special function to process it
        {
          code_.text.replace(ns_pose - 3, 5, "__ont().");
        }
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
  code_.operators_.describe("+=", "opAddAssign", true, 0);
  code_.operators_.describe("-=", "opSubAssign", true, 0);
  code_.operators_.describe("-", "opSub", false, 0);
  code_.operators_.describe("+", "opAdd", false, 0);
  code_.operators_.describe("=", "opAssign", true, 0);
  code_.operators_.dontCarre("==");
  code_.operators_.dontCarre("!=");
  code_.operators_.dontCarre("=if");

  code_.operators_.op2Function();
}
