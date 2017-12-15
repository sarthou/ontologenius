#include <iostream>

#include "ontoloGenius/Parser.h"
#include "ontoloGenius/Computer.h"

Parser::Parser(std::string code, TreeObject& onto, size_t current_line) : onto_(onto)
{
  code_ = code;
  parser_state_ = ParserState::wait;
  subparser_ = nullptr;
  begin_ = end_ = 0;

  lines_counter_.current_line_ = current_line;
  lines_counter_.setStart(current_line);

  std::cout << code_ << std::endl;
  std::cout << "----------------------" << std::endl;

  for (int i = 0; i < code_.length(); )
  {
    if(code_[i] == '\t')
      code_.erase(i, 1);
    else
      i++;
  }

  removeComments();

  getSubsections();

  getIfBlock();

  std::cout << code_ << std::endl;
  std::cout << "----------------------" << std::endl;
}

Parser::~Parser()
{
  if(subparser_ != nullptr)
    delete subparser_;
}

ParserState Parser::getState() const
{
  if(subparser_ != nullptr)
    return parser_state_;
  else
    return subparser_->getState();
}

size_t Parser::getInBraquet(size_t begin, std::string& in_braquet, std::string& text)
{
  size_t braquet = begin;
  while((text[braquet] == ' ') || (text[braquet] == '\n'))
    braquet += 1;

  if(text[braquet] == '(')
  {
    size_t first_braquet = braquet;
    int cpt = 1;
    while((cpt != 0) && (braquet+1 < text.length()))
    {
      ++braquet;
      if(text[braquet] == '(')
        cpt++;
      else if(text[braquet] == ')')
        cpt--;

    }

    in_braquet = text.substr(first_braquet+1, braquet-first_braquet-1);

    if(cpt == 0)
      return braquet;
    else
      return std::string::npos;
  }
  else
    return begin;
}

bool Parser::findBefore(size_t begin, char symbol)
{
  while((code_[begin-1] == ' ') || (code_[begin-1] == '\n'))
    begin -= 1;

  if(code_[begin-1] == symbol)
    return true;
  else
    return false;
}

/*
Return the position of the first caracter of the searched symbol if the symbol was found
return std::string::npos even else
/!\ begin can be on the last caracter of the precedent word
*/
size_t Parser::findAfter(size_t begin, std::string symbol)
{
  while((code_[begin+1] == ' ') || (code_[begin+1] == '\n'))
    begin += 1;

  size_t pose = code_.find(symbol, begin);

  if(pose == begin+1)
    return pose;
  else
    return std::string::npos;
}

/*
Return true the symbol was found
return false even else
/!\ begin must be on the first caracter
*/
bool Parser::findHere(size_t begin, std::string symbol)
{
  size_t pose = code_.find(symbol, begin);

  if(pose == begin)
    return true;
  else
    return false;
}

size_t Parser::getNbOfSublines(size_t& current_pose, size_t stop)
{
  bool eol = false;
  size_t nb_of_sublines = 0;

  while(eol == false)
  {
    if(current_pose > stop)
      break;
    else if(code_[current_pose] == '\0')
      return 0;
    else if(code_[current_pose] == '\n')
    {
      nb_of_sublines += 1;
      eol = true;
    }
    else if(code_[current_pose] == ' ')
    {}
    else if(findHere(current_pose, "__comment("))
    {
      size_t semicolon = code_.find(";", current_pose);
      nb_of_sublines += comments_[code_.substr(current_pose, semicolon-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = semicolon;
    }
    else if(findHere(current_pose, "__subsection("))
    {
      size_t semicolon = code_.find(";", current_pose);
      nb_of_sublines += subsections_[code_.substr(current_pose, semicolon-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = semicolon;
    }
    else if(findHere(current_pose, "__ifelse("))
    {
      size_t semicolon = code_.find(";", current_pose);
      nb_of_sublines += ifelse_[code_.substr(current_pose, semicolon-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = semicolon;
    }
    current_pose++;
  }

  return nb_of_sublines;
}

size_t Parser::getLineNumber(size_t final_pose)
{
  size_t current = lines_counter_.getStart();

  for(size_t i = 0; i < final_pose;)
    current += getNbOfSublines(i, final_pose);

  return current;
}

size_t Parser::getBeginOfLine(size_t line_nb)
{
  line_nb -= lines_counter_.getStart();

  size_t i = 0;
  for(; line_nb != 0; )
    line_nb -= getNbOfSublines(i);

  return i;
}

void Parser::printCursor(size_t pose)
{
  for(size_t i = 0; i < pose; i++)
    std::cout << " ";
  std::cout << "^" << std::endl;
}

void Parser::printError(size_t pose, std::string message)
{
  size_t line_error = getLineNumber(pose);
  size_t error_begin = getBeginOfLine(line_error);
  std::cout << "[" << line_error << ":" << (pose - error_begin + 1) << "] error: " << message << std::endl;
  size_t new_line = code_.find("\n", pose);
  std::string full_line = code_.substr(error_begin, new_line-error_begin);

  while(full_line.find("__comment(") != std::string::npos)
  {
    size_t comment_pose = full_line.find("__comment(");
    std::string comment_no;
    getInBraquet(comment_pose+9, comment_no, full_line);
    std::string comment = "__comment(" + comment_no + ");";
    full_line.replace(comment_pose, comment.size(), comments_[comment].comment );
    if(comment_pose + 1 < (pose - error_begin + 1))
      error_begin += comment.size() - comments_[comment].comment.size();
  }

  std::cout << full_line << std::endl;
  printCursor(pose - error_begin);
}

void Parser::removeComments()
{
  bool eof = false;
  uint16_t nb_comments = 0;

  do
  {
    size_t comment = code_.find("//", 0);
    if(comment == std::string::npos)
      eof = true;
    else
    {
      size_t new_line = code_.find("\n", comment);
      CommentBlock_t comment_i;
      comment_i.comment = code_.substr(comment, new_line-comment);
      comment_i.lines_count.setStart(getLineNumber(comment));
      comment_i.lines_count.setStop(comment_i.lines_count.getStart());
      comments_["__comment(" + std::to_string(nb_comments) + ");"] = comment_i;
      code_.replace(comment, new_line-comment, "__comment(" + std::to_string(nb_comments) + ");");

      nb_comments++;
    }
  }
  while(!eof);

  eof = false;
  do
  {
    size_t comment = code_.find("/*", 0);
    if(comment == std::string::npos)
      eof = true;
    else
    {
      size_t comment_end = code_.find("*/", comment+2);
      if(comment_end != std::string::npos)
      {
        CommentBlock_t comment_i;
        comment_i.comment = code_.substr(comment, comment_end-comment+2);
        comment_i.lines_count.setStart(getLineNumber(comment));
        comment_i.lines_count.setStop(getLineNumber(comment_end));
        comments_["__comment(" + std::to_string(nb_comments) + ");"] = comment_i;
        code_.replace(comment, comment_end-comment+2, "__comment(" + std::to_string(nb_comments) + ");");

        nb_comments++;
      }
      else
      {
        printError(comment, "expected ‘*/’ at end of input");
        eof = true;
      }
    }
  }
  while(!eof);

  eof = false;
  size_t bad_comment = 0;
  do
  {
    bad_comment = code_.find("*/", bad_comment);
    if(bad_comment == std::string::npos)
      eof = true;
    else
    {
      printError(bad_comment, "expected primary-expression before ‘*/’ token");
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
    size_t braquet = code_.find("{", begin);
    if(braquet == std::string::npos)
      eof = true;
    else
    {
      size_t braquet = code_.find("{", begin);
      size_t first_braquet = braquet;
      int cpt = 1;
      while((cpt != 0) && (code_[braquet] != '\0'))
      {
        ++braquet;
        if(code_[braquet] == '{')
          cpt++;
        else if(code_[braquet] == '}')
          cpt--;

      }

      if(cpt == 0)
      {
        SubsectionBlock_t subsection_i;
        subsection_i.subsection = code_.substr(first_braquet+1, braquet-first_braquet-1);
        subsection_i.lines_count.setStart(getLineNumber(first_braquet));
        subsection_i.lines_count.setStop(getLineNumber(braquet));

        subsections_["__subsection(" + std::to_string(nb_sub) + ");"] = subsection_i;
        code_.replace(first_braquet, braquet-first_braquet+1, "__subsection(" + std::to_string(nb_sub) + ");");

        nb_sub++;
      }
      else
      {
        printError(first_braquet, "expected ‘}’ at end of input");
        eof = true;
      }
    }
  }
  while(!eof);

  eof = false;
  size_t bad_subsection = 0;
  do
  {
    bad_subsection = code_.find("}", bad_subsection);
    if(bad_subsection == std::string::npos)
      eof = true;
    else
    {
      printError(bad_subsection, "expected primary-expression before ‘}’ token");
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
    size_t else_error = code_.find("else", pose);
    if(else_error == std::string::npos)
      eof = true;
    else
    {
      if(((code_[else_error - 1] == ' ') || (code_[else_error - 1] == '\n') || (code_[else_error - 1] == ';')) &&
        ((code_[else_error + 4] == ' ') || (code_[else_error + 4 ] == '\n') || (code_[else_error + 4] == ';')))
        {
          printError(else_error, "‘else’ without a previous ‘if’");
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

  size_t if_start = code_.find("if", pose);
  if(if_start == std::string::npos)
    return std::string::npos;
  else
  {
    IfBlock_t if_block;
    pose = getInBraquet(if_start+2, if_block.IfBlock_condition, code_);

    if(pose == if_start+2)
    {
      printError(pose, "expected ‘(’ after 'if’");
      return std::string::npos;
    }
    else if(pose == std::string::npos)
    {
      printError(if_start+2, "expected corresponding ‘)’ after previous '(’");
      return std::string::npos;
    }

    if(findAfter(pose, "if") != std::string::npos)
      getNextIfBlock(nb_block, pose);

    size_t semicolon = code_.find(";", pose);
    if_block.IfBlock_if = code_.substr(pose+1, semicolon-pose);
    if_block.lines_count.setStart(getLineNumber(if_start));
    if_block.lines_count.setStop(getLineNumber(semicolon));
    pose = semicolon;
    end = semicolon;

    if(findBefore(if_start, '='))
      return code_.find(";", if_start);

    size_t else_start = findAfter(pose, "else");
    if(else_start != std::string::npos)
    {
      if(findAfter(else_start+4, "if") != std::string::npos)
        getNextIfBlock(nb_block, else_start);

      semicolon = code_.find(";", else_start);
      if_block.IfBlock_else = code_.substr(else_start+4, semicolon-else_start-4+1);
      if_block.lines_count.setStop(getLineNumber(semicolon));
      pose = semicolon;
      end = semicolon;
    }

    ifelse_["__ifelse(" + std::to_string(nb_block) + ");"] = if_block;
    code_.replace(if_start, end-if_start+1, "__ifelse(" + std::to_string(nb_block) + ");");

    pose -= (end-if_start+1) - std::string("__ifelse(" + std::to_string(nb_block) + ");").size();

    nb_block++;
  }

  return pose;
}
