#include "ontologenius/interpreter/codeDescription/TextManipulator.h"

namespace ontologenius {

size_t TextManipulator::getInBraquet(size_t begin, std::string& in_bracket, std::string& text)
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

bool TextManipulator::findBefore(size_t begin, char symbol)
{
  while((text[begin-1] == ' ') || (text[begin-1] == '\n'))
    begin -= 1;

  if(text[begin-1] == symbol)
    return true;
  else
    return false;
}

bool TextManipulator::findJustBefore(size_t begin, std::string symbol)
{
  for(size_t i = 0; i < symbol.size(); i++)
  {
    if(text[begin - 1 - i] != symbol[symbol.size() - 1 - i])
      return false;
  }
  return true;
}

std::string TextManipulator::getWordBefore(size_t begin)
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

std::string TextManipulator::getWordAfter(size_t begin, bool just_after)
{
  if(just_after == false)
    while((text[begin+1] == ' ') || (text[begin+1] == '\n'))
      begin += 1;

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
size_t TextManipulator::findAfter(size_t begin, std::string symbol)
{
  while((text[begin+1] == ' ') || (text[begin+1] == '\n'))
    begin += 1;

  size_t pose = text.find(symbol, begin);

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
bool TextManipulator::findHere(size_t begin, std::string symbol)
{
  size_t pose = text.find(symbol, begin);

  if(pose == begin)
    return true;
  else
    return false;
}

void TextManipulator::remove(char character)
{
  for(size_t i = 0; i < text.length(); )
  {
    if(text[i] == character)
      text.erase(i, 1);
    else
      i++;
  }
}

} // namespace ontologenius
