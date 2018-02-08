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
    else if(findHere(current_pose, "__comment["))
    {
      size_t bracket = text.find("]", current_pose);
      nb_of_sublines += comments_[text.substr(current_pose, bracket-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = bracket;
    }
    else if(findHere(current_pose, "__subsection["))
    {
      size_t bracket = text.find("]", current_pose);
      nb_of_sublines += subsections_[text.substr(current_pose, bracket-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = bracket;
    }
    else if(findHere(current_pose, "__ifelse["))
    {
      size_t semicolon = text.find(";", current_pose);
      nb_of_sublines += ifelse_[text.substr(current_pose, semicolon-current_pose+1)].lines_count.getNbLines() - 1;
      current_pose = semicolon;
    }
    else if(findHere(current_pose, "__string["))
    {
      size_t bracket = text.find("]", current_pose);
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
    pose += useless;
    code = code.substr(useless);

    size_t comment = code.find("__comment[");
    if(comment == 0)
    {
      done = false;
      comment = code.find("]");
      pose += comment + 1;
      code = code.substr(comment + 1);
    }
  }
}

void Code::removeNonEffectiveCode(std::string& code)
{
  bool done = false;

  while(done == false)
  {
    done = true;
    int16_t useless = code.size() - 1;
    while(((code[useless] == ' ') || (code[useless] == '\n')) && (useless >= 0))
      useless -= 1;
    code = code.substr(0,useless+1);

    size_t tmp_useless = code.size() - 1;
    if(code[tmp_useless] ==']')
    {
      while(code[--tmp_useless] != '[');
      if(code.find("__comment[", tmp_useless - 9) != std::string::npos)
      {
        code = code.substr(0, tmp_useless - 9);
        done = false;
      }
    }
  }
}

size_t Code::checkWordIntegrity(std::string& wholeWord)
{
  std::string tmp_wholeWord = wholeWord;

  size_t offset = 0;
  removeNonEffectiveCode(tmp_wholeWord);
  goToEffectiveCode(tmp_wholeWord, offset);

  size_t i = 0;

  while((i < tmp_wholeWord.size()) && ((tmp_wholeWord[i] >= '0' && tmp_wholeWord[i] <= '9') ||
                                      (tmp_wholeWord[i] >= 'A' && tmp_wholeWord[i] <= 'Z') ||
                                      (tmp_wholeWord[i] >= 'a' && tmp_wholeWord[i] <= 'z') ||
                                      (tmp_wholeWord[i] == '_')))
  {
    i++;
  }
  std::string validWord = tmp_wholeWord.substr(0, i);
  if(tmp_wholeWord == validWord)
    return std::string::npos;
  else
  {
    wholeWord=validWord;
    return i+offset;
  }
}

void Code::removeProtectedWord(std::string& text)
{
  size_t protect = text.find("__");
  if(protect != std::string::npos)
    text = text.substr(0, protect);
}
