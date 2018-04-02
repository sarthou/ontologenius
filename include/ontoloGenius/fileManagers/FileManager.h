#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include <string>

class FileManager
{
public:
  FileManager();
  ~FileManager();

  void init(std::string file_name, std::string option);
  void reset(std::string file_name);
protected:
  FILE* file_;
};

#endif
