#include <iostream>

#include "ontoloGenius/Error.h"

#define COLOR_OFF     "\x1B[0m"
#define COLOR_RED     "\x1B[0;91m"
#define COLOR_ORANGE  "\x1B[0;31m"
#define COLOR_GREEN   "\x1B[1;92m"

void Error::printError(size_t pose, std::string message)
{
  std::string msg = std::string(COLOR_RED + std::string("error: ") + COLOR_OFF + message);
  printMessage(pose, msg);
}

void Error::printWarning(size_t pose, std::string message)
{
  std::string msg = std::string(COLOR_ORANGE + std::string("warning: ") + COLOR_OFF + message);
  printMessage(pose, msg);
}

size_t Error::getBeginOfLine(size_t line_nb)
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

void Error::printCursor(size_t pose)
{
  for(size_t i = 0; i < pose; i++)
    std::cout << " ";
  std::cout << COLOR_GREEN << "^" << COLOR_OFF << std::endl;
}

void Error::printMessage(size_t pose, std::string message)
{
  size_t line_error = code_->getLineNumber(pose);
  size_t error_begin = getBeginOfLine(line_error);
  size_t new_line = code_->text.find("\n", pose);
  std::string full_line = code_->text.substr(error_begin, new_line-error_begin);

  while(full_line.find("__subsection(") != std::string::npos)
  {
    size_t subsection_pose = full_line.find("__subsection(");
    std::string subsection_no;
    code_->getInBraquet(subsection_pose+12, subsection_no, full_line);
    std::string subsection = "__subsection(" + subsection_no + ");";
    full_line.replace(subsection_pose, subsection.size(), std::string("{" + code_->subsections_[subsection].subsection + "}"));
    if(subsection_pose + 1 < (pose - error_begin + 1))
      pose += std::string("{" + code_->subsections_[subsection].subsection + "}").size() - subsection.size();
  }

  while(full_line.find("__comment(") != std::string::npos)
  {
    size_t comment_pose = full_line.find("__comment(");
    std::string comment_no;
    code_->getInBraquet(comment_pose+9, comment_no, full_line);
    std::string comment = "__comment(" + comment_no + ");";
    full_line.replace(comment_pose, comment.size(), code_->comments_[comment].comment );
    if(comment_pose + 1 < (pose - error_begin + 1))
      pose += code_->comments_[comment].comment.size() - comment.size();
  }

  while(full_line.find("__string(") != std::string::npos)
  {
    size_t string_pose = full_line.find("__string(");
    std::string string_no;
    code_->getInBraquet(string_pose+8, string_no, full_line);
    std::string strings = "__string(" + string_no + ")";

    full_line.replace(string_pose, strings.size(), std::string("\"" + code_->strings_[strings].strings + "\""));
    if(string_pose + 1 < (pose - error_begin + 1))
      pose += std::string("\"" + code_->strings_[strings].strings + "\"").size() - strings.size();
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
}
