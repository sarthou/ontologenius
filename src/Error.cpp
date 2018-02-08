#include <iostream>

#include "ontoloGenius/Error.h"

#define COLOR_OFF     "\x1B[0m"
#define COLOR_RED     "\x1B[0;91m"
#define COLOR_ORANGE  "\x1B[1;33m"
#define COLOR_GREEN   "\x1B[1;92m"

Error::Error(Code* code)
{
  code_ = code;
  nb_error = 0;
  nb_wrng = 0;
}

void Error::printError(size_t pose, std::string message)
{
  std::string msg = std::string(COLOR_RED + std::string("error: ") + COLOR_OFF + message);
  printMessage(pose, msg);
  nb_error++;
}

void Error::printWarning(size_t pose, std::string message)
{
  std::string msg = std::string(COLOR_ORANGE + std::string("warning: ") + COLOR_OFF + message);
  printMessage(pose, msg);
  nb_wrng++;
}

void Error::printStatus()
{
  if(nb_error)
    std::cout << COLOR_RED;
  std::cout << nb_error << " errors " << COLOR_OFF << "and ";

  if(nb_wrng)
    std::cout << COLOR_ORANGE;
  std::cout << nb_wrng << " warnings " << COLOR_OFF << std::endl;

  if(nb_error)
    std::cout << COLOR_RED << std::string("pre-processing failed") << COLOR_OFF << std::endl;
  else
    std::cout << COLOR_GREEN << std::string("pre-processing succeed") << COLOR_OFF << std::endl;
}

bool Error::isOnError()
{
  if(nb_error)
    return true;
  else
    return false;
}

void Error::cpy(Error& error)
{
  nb_error = error.nb_error;
  nb_wrng = error.nb_wrng;
}

size_t Error::getBeginOfLine(size_t line_nb)
{
  if(code_ != nullptr)
  {
    line_nb -= code_->lines_counter_.getStart();

    int nb_line  = line_nb;

    size_t i = 0;
    size_t savedN = i;
    size_t savedN_1 = i;
    for(; nb_line > 0; )
    {
      size_t nb_sublines = code_->getNbOfSublines(i);
      if(nb_sublines != 0)
      {
        nb_line -= nb_sublines;
        savedN_1 = savedN;
        savedN = i;
      }
    }

    if(nb_line < 0)
      i = savedN_1;

    return i;
  }
  else return 0;
}

void Error::printCursor(size_t pose)
{
  for(size_t i = 0; i < pose; i++)
    std::cout << " ";
  std::cout << COLOR_GREEN << "^" << COLOR_OFF << std::endl;
}

void Error::printMessage(size_t pose, std::string message)
{
  if(code_ != nullptr)
  {
    bool uncompact = code_->ifelse_.uncompact(*code_);

    size_t line_error = code_->getLineNumber(pose);
    size_t error_begin = getBeginOfLine(line_error);
    size_t new_line = code_->text.find("\n", pose);
    std::string full_line = code_->text.substr(error_begin, new_line-error_begin);

    code_->operators_.function2Op(full_line, error_begin, pose);

    while(full_line.find("__ont") != std::string::npos)
    {
      size_t ont_pose = full_line.find("__ont");
      full_line.replace(ont_pose, 6, "ont::");
      if(ont_pose + 1 < (pose - error_begin + 1))
        pose += std::string("ont::").size() - std::string("__ont.").size();
    }

    while(full_line.find("__var[") != std::string::npos)
    {
      size_t var_pose = full_line.find("__var[");
      size_t var_end = full_line.find("]", var_pose);
      std::string var = full_line.substr(var_pose, var_end-var_pose+1);
      std::string ns_var = code_->variables_.ns() + "::" + code_->variables_.name(var);
      full_line.replace(var_pose, var.size(), ns_var);
      if(var_pose + 1 < (pose - error_begin + 1))
        pose += ns_var.size() - var.size();
    }

    while(full_line.find("__subsection[") != std::string::npos)
    {
      size_t subsection_pose = full_line.find("__subsection[");
      size_t subsection_end = full_line.find("]", subsection_pose);
      std::string subsection = full_line.substr(subsection_pose, subsection_end-subsection_pose+1+1);
      full_line.replace(subsection_pose, subsection.size(), std::string("{" + code_->subsections_[subsection].subsection->text + "}"));
      if(subsection_pose + 1 < (pose - error_begin + 1))
        pose += std::string("{" + code_->subsections_[subsection].subsection->text + "}").size() - subsection.size();
    }

    while(full_line.find("__comment[") != std::string::npos)
    {
      size_t comment_pose = full_line.find("__comment[");
      size_t comment_end = full_line.find("]", comment_pose);
      std::string comment = full_line.substr(comment_pose, comment_end-comment_pose+1);
      full_line.replace(comment_pose, comment.size(), code_->comments_[comment].comment );
      if(comment_pose + 1 < (pose - error_begin + 1))
        pose += code_->comments_[comment].comment.size() - comment.size();
    }

    while(full_line.find("__string[") != std::string::npos)
    {
      size_t string_pose = full_line.find("__string[");
      size_t string_end = full_line.find("]", string_pose);
      std::string strings = full_line.substr(string_pose, string_end-string_pose+1);
      full_line.replace(string_pose, strings.size(), std::string("\"" + code_->strings_.get(strings) + "\""));
      if(string_pose + 1 < (pose - error_begin + 1))
        pose += std::string("\"" + code_->strings_.get(strings) + "\"").size() - strings.size();
    }

    for(size_t i = (pose - error_begin + 1); i < full_line.size(); i++)
    {
      if(full_line[i] == '\n')
      {
        full_line = full_line.substr(0, i);
        break;
      }
    }

    while(full_line.find("\n") != std::string::npos)
    {
      size_t newline_pose = full_line.find("\n");
      if(newline_pose < error_begin)
      {
        full_line = full_line.substr(newline_pose+1, full_line.size() - (newline_pose+1));
        error_begin = error_begin + newline_pose + 1;
      }
    }

    std::cout << "[" << line_error << ":" << (pose - error_begin + 1) << "] " << message << std::endl;
    std::cout << full_line << std::endl;
    printCursor(pose - error_begin);

    if(uncompact)
      code_->ifelse_.compact(*code_);
  }
}
