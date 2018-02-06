#include "ontoloGenius/codeDescription/Code.h"

size_t Code::getLineNumber(size_t final_pose)
{
  size_t current = lines_counter_.getStart();

  for(size_t i = 0; i < final_pose;)
    current += getNbOfSublines(i, final_pose);

  return current;
}

size_t Code::getNbOfSublines(size_t& current_pose, size_t stop)
{
  bool eol = false;
  size_t nb_of_sublines = 0;

  while(eol == false)
  {
    if(current_pose >= stop)
      break;
    else if(text[current_pose] == '\0')
      return 0;
    else if(text[current_pose] == '\n')
    {
      nb_of_sublines += 1;
      eol = true;
    }
    else if(text[current_pose] == ' ')
    {}
    else if(findHere(current_pose, "__comment("))
    {
      size_t bracket = text.find(")", current_pose);
      nb_of_sublines += comments_[text.substr(current_pose, bracket-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = bracket;
    }
    else if(findHere(current_pose, "__subsection("))
    {
      size_t bracket = text.find(")", current_pose);
      nb_of_sublines += subsections_[text.substr(current_pose, bracket-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = bracket;
    }
    else if(findHere(current_pose, "__ifelse("))
    {
      size_t semicolon = text.find(";", current_pose);
      nb_of_sublines += ifelse_[text.substr(current_pose, semicolon-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = semicolon;
    }
    else if(findHere(current_pose, "__string("))
    {
      size_t bracket = text.find(")", current_pose);
      nb_of_sublines += strings_.nbLines(text.substr(current_pose, bracket-current_pose+1)) - 1;
      current_pose = bracket;
    }
    current_pose++;
  }

  return nb_of_sublines;
}

void Code::goToEffectiveCode(std::string& code, size_t& pose)
{
  bool done = false;

  while(done == false)
  {
    done = true;
    size_t useless = 0;
    while((code[useless] == ' ') || (code[useless] == '\n'))
      useless += 1;
    pose += useless; //TODO remove if ok
    code = code.substr(useless);

    size_t comment = code.find("__comment(");
    if(comment == 0)
    {
      done = false;
      comment = code.find(")");
      pose += comment + 1;
      code = code.substr(comment + 1);
    }
  }
}
