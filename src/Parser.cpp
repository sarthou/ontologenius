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

  getStrings();

  removeComments();

  getSubsections();

  getFromNamespace();

  //getIfBlock();

  code_.print();
  std::cout << "----------------------" << std::endl;

  error_.printStatus();
}

Parser::~Parser()
{
  if(subparser_ != nullptr)
    delete subparser_;
}

void Parser::removeComments()
{
  bool eof = false;
  uint16_t nb_comments = 0;

  do
  {
    size_t comment = code_.text.find("//", 0);
    if(comment == std::string::npos)
      eof = true;
    else
    {
      size_t new_line = code_.text.find("\n", comment);
      CommentBlock_t comment_i;
      comment_i.comment = code_.text.substr(comment, new_line-comment);
      comment_i.lines_count.setStart(code_.getLineNumber(comment));
      comment_i.lines_count.setStop(comment_i.lines_count.getStart());
      code_.comments_["__comment(" + std::to_string(nb_comments) + ");"] = comment_i;
      code_.text.replace(comment, new_line-comment, "__comment(" + std::to_string(nb_comments) + ");");

      nb_comments++;
    }
  }
  while(!eof);

  eof = false;
  do
  {
    size_t comment = code_.text.find("/*", 0);
    if(comment == std::string::npos)
      eof = true;
    else
    {
      size_t comment_end = code_.text.find("*/", comment+2);
      if(comment_end != std::string::npos)
      {
        CommentBlock_t comment_i;
        comment_i.comment = code_.text.substr(comment, comment_end-comment+2);
        comment_i.lines_count.setStart(code_.getLineNumber(comment));
        comment_i.lines_count.setStop(code_.getLineNumber(comment_end));
        code_.comments_["__comment(" + std::to_string(nb_comments) + ");"] = comment_i;
        code_.text.replace(comment, comment_end-comment+2, "__comment(" + std::to_string(nb_comments) + ");");

        nb_comments++;
      }
      else
      {
        error_.printError(comment, "expected ‘*/’ at end of input");
        eof = true;
      }
    }
  }
  while(!eof);

  eof = false;
  size_t bad_comment = 0;
  do
  {
    bad_comment = code_.text.find("*/", bad_comment);
    if(bad_comment == std::string::npos)
      eof = true;
    else
    {
      error_.printError(bad_comment, "expected primary-expression before ‘*/’ token");
      bad_comment++;
    }
  }
  while(!eof);
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
    size_t braquet = code_.text.find("{", begin);
    if(braquet == std::string::npos)
      eof = true;
    else
    {
      size_t braquet = code_.text.find("{", begin);
      size_t first_braquet = braquet;
      int cpt = 1;
      while((cpt != 0) && (code_.text[braquet] != '\0'))
      {
        ++braquet;
        if(code_.text[braquet] == '{')
          cpt++;
        else if(code_.text[braquet] == '}')
          cpt--;

      }

      if(cpt == 0)
      {
        SubsectionBlock_t subsection_i;
        subsection_i.subsection = code_.text.substr(first_braquet+1, braquet-first_braquet-1);
        subsection_i.lines_count.setStart(code_.getLineNumber(first_braquet));
        subsection_i.lines_count.setStop(code_.getLineNumber(braquet));

        code_.subsections_["__subsection(" + std::to_string(nb_sub) + ");"] = subsection_i;
        code_.text.replace(first_braquet, braquet-first_braquet+1, "__subsection(" + std::to_string(nb_sub) + ");");

        nb_sub++;
      }
      else
      {
        error_.printError(first_braquet, "expected ‘}’ at end of input");
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

void Parser::getStrings()
{
  bool eof = false;
  uint16_t nb_strings = 0;

  do
  {
    size_t strings = code_.text.find("\"", 0);
    if(strings == std::string::npos)
      eof = true;
    else
    {
      size_t string_end = code_.text.find("\"", strings+1);
      if(string_end != std::string::npos)
      {
        StringBlock_t string_i;
        string_i.strings = code_.text.substr(strings+1, string_end-strings - 1);
        string_i.lines_count.setStart(code_.getLineNumber(strings));
        string_i.lines_count.setStop(code_.getLineNumber(string_end));
        code_.strings_["__string(" + std::to_string(nb_strings) + ")"] = string_i;
        code_.text.replace(strings, string_end-strings+1, "__string(" + std::to_string(nb_strings) + ")");

        nb_strings++;
      }
      else
      {
        error_.printError(strings, "expected ‘\"’ at end of input");
        eof = true;
      }
    }
  }
  while(!eof);
}

void Parser::getFromNamespace()
{
  bool eof = false;
  uint16_t nb_var = 0;
  uint16_t nb_func = 0;

  do
  {
    size_t ns_pose = code_.text.find(":", 0);
    if(ns_pose == std::string::npos)
      eof = true;
    else
    {
      if(code_.findHere(ns_pose+1, ":"))
      {
        if(code_.findJustBefore(ns_pose, "var"))
        {
          std::string var = code_.getWordAfter(ns_pose+1);
          std::string var_id = code_.variables_.add(var);
          code_.text.replace(ns_pose - 3, var.size() + 5, var_id);
        }
        else if(code_.findJustBefore(ns_pose, "ont")) //TODO : create special function to process it
        {
          FunctionBlock_t block;
          block.name = code_.getWordAfter(ns_pose+1);
          std::string func = "__func(" + std::to_string(nb_func) + ")";
          code_.functions_[func] = block;
          code_.text.replace(ns_pose - 3, block.name.size() + 5, func);
          nb_func++;
        }
        else
        {
          std::string bad_ns = code_.getWordBefore(ns_pose);
          error_.printError(ns_pose - bad_ns.size(), "unknow namespace '" + bad_ns + "'");
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
