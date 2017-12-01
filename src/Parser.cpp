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

  removeComments();

  for (int i = 0; i < code_.length(); )
  {
    if(code_[i] == '\t')
      code_.erase(i, 1);
    else
      i++;
  }

  getSubsections();

  std::cout << code_ << std::endl;
  std::cout << "----------------------" << std::endl;

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

bool Parser::move()
{
  if((parser_state_ == ParserState::instruction) || (parser_state_ == ParserState::loop))
    parser_state_ = ParserState::wait;

  size_t find_semicolon = code_.find(";", begin_);
  size_t find_if = code_.find("if", begin_);
  size_t find_egual_if = code_.find("=if", begin_);

  if(find_semicolon < find_if)
  {
    end_ = find_semicolon;
    instruction();
  }
  else if(find_egual_if < find_if)
  {
    end_ = find_semicolon;
    loop();
  }
  else if(find_if != std::string::npos)
    if_condition();
  else
    return false;

  begin_ = end_+1;
  return true;
}

void Parser::loop()
{
  parser_state_ = ParserState::loop;
  std::cout << "[loop]" << code_.substr(begin_, end_-begin_+1) << std::endl;
}

void Parser::instruction()
{
  if(parser_state_ != ParserState::condition_if_false)
  {
    parser_state_ = ParserState::instruction;
    std::string instruction = code_.substr(begin_, end_-begin_+1);
    std::cout << "[inst]" << instruction << std::endl;
    if(instruction.find("__subsection") != std::string::npos)
    {
      Parser p(subsections_[instruction].subsection, onto_);
      std::cout << "{" << std::endl;
      while(p.move());
      std::cout << "}" << std::endl;
    }
  }
}

bool Parser::if_condition()
{
  parser_state_ = ParserState::condition_if;
  size_t braquet = code_.find("(", begin_);
  size_t first_braquet = braquet;
  int cpt = 1;
  while(cpt != 0)
  {
    ++braquet;
    if(code_[braquet] == '(')
      cpt++;
    else if(code_[braquet] == ')')
      cpt--;

  }
  end_ = braquet;

  std::cout << "[if]" << code_.substr(begin_, end_-begin_+1) << std::endl;

  bool result = true;
  Computer comp;
  //result = comp.compute(code_.substr(first_braquet+1, end_-first_braquet-1), onto_);
  if(result)
    parser_state_ = ParserState::condition_if_true;
  else
    parser_state_ = ParserState::condition_if_false;

  return result;
}

size_t Parser::getInBraquet(size_t begin, std::string& in_braquet)
{
  size_t braquet = code_.find("(", begin);
  size_t first_braquet = braquet;
  int cpt = 1;
  while(cpt != 0)
  {
    ++braquet;
    if(code_[braquet] == '(')
      cpt++;
    else if(code_[braquet] == ')')
      cpt--;

  }
  braquet;

  in_braquet = code_.substr(first_braquet+1, braquet-first_braquet-1);
  return braquet;
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

size_t Parser::getLineNumber(size_t final_pose)
{
  size_t current = lines_counter_.getStart();
  for(size_t i = 0; i < final_pose; i++)
  {
    if(code_[i] == '\n')
      current++;
    else if(code_[i] == ' ')
      continue;
    else if(findAfter(i, "__comment(") != std::string::npos)
    {
      size_t semicolon = code_.find(";", i);
      current += comments_[code_.substr(i, semicolon-i+1)].lines_count.getNbLines();
      i += semicolon-i;
    }
  }

  return current;
}

size_t Parser::getBeginOfLine(size_t line_nb)
{
  line_nb -= lines_counter_.getStart();

  size_t i = 0;
  for(; line_nb != 0; i++)
  {
    if(code_[i] == '\n')
      line_nb--;
    else if(code_[i] == ' ')
      continue;
    else if(findAfter(i, "__comment(") != std::string::npos)
    {
      size_t semicolon = code_.find(";", i);
      line_nb -= comments_[code_.substr(i, semicolon-i+1)].lines_count.getNbLines();
      i += semicolon-i;
    }
  }
  return i;
}

void Parser::printCursor(size_t pose)
{
  for(size_t i = 0; i < pose; i++)
    std::cout << " ";
  std::cout << "^" << std::endl;
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
        size_t line_error = getLineNumber(comment);
        size_t error_begin = getBeginOfLine(line_error);
        std::cout << "[" << line_error << ":" << (comment - error_begin + 1) << "] error: expected ‘*/’ at end of input" << std::endl;
        size_t new_line = code_.find("\n", comment);
        std::cout << code_.substr(error_begin, new_line-error_begin) << std::endl;
        printCursor(comment - error_begin);

        eof = true;
      }
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
        size_t line_error = getLineNumber(first_braquet);
        size_t error_begin = getBeginOfLine(line_error);
        std::cout << "[" << line_error << ":" << (first_braquet - error_begin + 1) << "] error: expected ‘}’ at end of input" << std::endl;
        size_t new_line = code_.find("\n", first_braquet);
        std::cout << code_.substr(error_begin, new_line-error_begin) << std::endl;
        printCursor(first_braquet - error_begin);

        eof = true;
      }
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
    pose = getInBraquet(if_start, if_block.IfBlock_condition);

    if(findAfter(pose, "if") != std::string::npos)
      getNextIfBlock(nb_block, pose);

    size_t semicolon = code_.find(";", pose);
    if_block.IfBlock_if = code_.substr(pose+1, semicolon-pose);
    pose = semicolon;
    end = semicolon;

    if(findBefore(if_start, '='))
      return code_.find(";", if_start);

    size_t else_start = findAfter(pose, "else");
    if(else_start != std::string::npos)
    {
      if(findAfter(else_start+4, "if") != std::string::npos)
        getNextIfBlock(nb_block, else_start);
      else
        std::cout << "i don't find" << std::endl;

      semicolon = code_.find(";", else_start);
      if_block.IfBlock_else = code_.substr(else_start+4, semicolon-else_start-4+1);
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
