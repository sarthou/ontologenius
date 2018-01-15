#include "ontoloGenius/codeDescription/Code.h"

size_t Code::getInBraquet(size_t begin, std::string& in_bracket, std::string& text)
{
  size_t bracket = begin;
  while((text[bracket] == ' ') || (text[bracket] == '\n'))
    bracket += 1;

  if(text[bracket] == '(')
  {
    size_t first_bracket = bracket;
    int cpt = 1;
    while((cpt != 0) && (bracket+1 < text.length()))
    {
      ++bracket;
      if(text[bracket] == '(')
        cpt++;
      else if(text[bracket] == ')')
        cpt--;

    }

    in_bracket = text.substr(first_bracket+1, bracket-first_bracket-1);

    if(cpt == 0)
      return bracket;
    else
      return std::string::npos;
  }
  else
    return begin;
}

bool Code::findBefore(size_t begin, char symbol)
{
  while((text[begin-1] == ' ') || (text[begin-1] == '\n'))
    begin -= 1;

  if(text[begin-1] == symbol)
    return true;
  else
    return false;
}

bool Code::findJustBefore(size_t begin, std::string symbol)
{
  for(int i = 0; i < symbol.size(); i++)
  {
    if(text[begin - 1 - i] != symbol[symbol.size() - 1 - i])
      return false;
  }
  return true;
}

std::string Code::getWordBefore(size_t begin)
{
  size_t i = begin;
  while((i != 0) && ((text[i -1] >= '0' && text[i-1] <= '9') ||
                      (text[i-1] >= 'A' && text[i-1] <= 'Z') ||
                      (text[i-1] >= 'a' && text[i-1] <= 'z') ||
                      (text[i-1] == '_')))
  {
    i--;
  }
  std::string result = text.substr(i, begin - i);
  return result;
}

std::string Code::getWordAfter(size_t begin)
{
  size_t i = begin;
  while((i != text.size() - 1) && ((text[i+1] >= '0' && text[i+1] <= '9') ||
                                    (text[i+1] >= 'A' && text[i+1] <= 'Z') ||
                                    (text[i+1] >= 'a' && text[i+1] <= 'z') ||
                                    (text[i+1] == '_')))
  {
    i++;
  }
  std::string result = text.substr(begin + 1, i - begin);
  return result;
}

/*
Return the position of the first caracter of the searched symbol if the symbol was found
return std::string::npos even else
/!\ begin can be on the last caracter of the precedent word
*/
size_t Code::findAfter(size_t begin, std::string symbol)
{
  while((text[begin+1] == ' ') || (text[begin+1] == '\n'))
    begin += 1;

  size_t pose = text.find(symbol, begin);

  if(pose == begin+1)
    return pose;
  else
    return std::string::npos;
}

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

/*
Return true the symbol was found
return false even else
/!\ begin must be on the first caracter
*/
bool Code::findHere(size_t begin, std::string symbol)
{
  size_t pose = text.find(symbol, begin);

  if(pose == begin)
    return true;
  else
    return false;
}

void Code::remove(char character)
{
  for (int i = 0; i < text.length(); )
  {
    if(text[i] == character)
      text.erase(i, 1);
    else
      i++;
  }
}
