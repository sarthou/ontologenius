#include "ontoloGenius/interpreter/fileManagers/FileManager.h"

#include <iostream>

#include <ros/package.h>

namespace ontologenius {

FileManager::FileManager()
{
  file_ = NULL;
}

FileManager::~FileManager()
{
  if(file_ != NULL)
    fclose(file_);
}

void FileManager::init(std::string file_name, std::string option)
{
  std::string path = ros::package::getPath("ontologenius");
  path+= "/build/" + file_name + ".ont";

  file_ = fopen(path.c_str(), option.c_str());
  if(file_ == NULL)
    std::cout << "Fail to open file " << file_name << " with option '" << option << "'" << std::endl;
}

void FileManager::reset(std::string file_name)
{
  std::string path = ros::package::getPath("ontologenius");
  path+= "/build/" + file_name + ".ont";

  file_ = fopen(path.c_str(), "w");
  if(file_ == NULL)
    std::cout << "Fail to reset file " << file_name << std::endl;
  else
    fclose(file_);
  file_ = NULL;
}

} // namespace ontologenius
