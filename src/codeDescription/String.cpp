#include "ontoloGenius/codeDescription/String.h"

String::String()
{
  nb_strings_ = 0;
}

std::string String::add(std::string text, size_t line_start, size_t line_stop)
{
  std::string id = "__string(" + std::to_string(nb_strings_) + ")";
  nb_strings_++;

  StringBlock_t tmp;
  tmp.strings = text;
  tmp.lines_count.setStart(line_start);
  tmp.lines_count.setStop(line_stop);
  strings_[id] = tmp;

  return id;
}

std::string String::get(std::string id)
{
  if(strings_.find(id) != strings_.end())
    return strings_[id].strings;
  return "";
}

size_t String::nbLines(std::string id)
{
  if(strings_.find(id) != strings_.end())
    return strings_[id].lines_count.getNbLines();
  return 0;
}

size_t String::size(std::string id)
{
  if(strings_.find(id) != strings_.end())
    return strings_[id].strings.size();
  return 0;
}
