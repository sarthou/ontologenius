#include "ontoloGenius/core/reasoner/ConfigReader.h"

#include <iostream>
#include <fstream>
#include <regex>

namespace ontologenius
{

bool ConfigReader::read(const std::string& path)
{
  std::ifstream config_file(path);
  if(config_file.is_open())
  {
    std::regex element_regex("^\\s*([^\\s]*)\\s*:\\s*([^\\n]*)\\s*$");
    std::regex list_regex("^\\s*-\\s*(.*)\\s*$");
    std::smatch match;

    std::string line;
    std::string element;
    while(std::getline(config_file,line))
    {
      removeComment(line);
      if(std::regex_match(line, match, element_regex))
      {
        if(match[2].str() == "")
        {
          config_[match[1].str()] = ConfigElement();
          element = match[1].str();
        }
        else
        {
          auto tmp = std::pair<std::string, ConfigElement>(match[1].str(), ConfigElement());
          if(!config_[element].subelem)
            config_[element].subelem = std::map<std::string, ConfigElement>();
          config_[element].subelem->insert(tmp);
          if(!config_[element].subelem->at(match[1].str()).data)
            config_[element].subelem->at(match[1].str()).data = std::vector<std::string>();
          config_[element].subelem->at(match[1].str()).data->push_back(match[2].str());
        }
      }
      else if(std::regex_match(line, match, list_regex))
      {
        if(!config_[element].data)
          config_[element].data = std::vector<std::string>();
        config_[element].data->push_back(match[1].str());
      }
    }
    config_file.close();
    return true;
  }
  else
    return false;
}

void ConfigReader::display()
{
  display(config_);
}

void ConfigReader::display(std::map<std::string, ConfigElement>& config, size_t nb)
{
  for(auto c : config)
  {
    displayTab(nb);
    std::cout << c.first << " : " << std::endl;
    if(c.second.data)
    {
      for(auto d : c.second.data.value())
      {
        displayTab(nb+1);
        std::cout << "- " << d  << std::endl;
      }
    }
    else if(c.second.subelem)
      display(c.second.subelem.value(), nb+1);
  }
}

void ConfigReader::displayTab(size_t nb)
{
  for(size_t i = 0; i < nb; i++)
    std::cout << "\t";
}

void ConfigReader::removeComment(std::string& line)
{
  size_t pose = line.find("#");
  if(pose != std::string::npos)
    line = line.substr(0, pose);
}

} // namespace ontologenius
