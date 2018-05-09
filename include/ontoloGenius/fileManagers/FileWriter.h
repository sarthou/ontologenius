#ifndef FILEWRITER_H
#define FILEWRITER_H

#include "ontoloGenius/fileManagers/FileManager.h"

#include <string>

class FileWriter : public FileManager
{
public:
  FileWriter() {}
  ~FileWriter() {}

  void write(std::string text);
  void writeLine(std::string text) { write(text + "\n"); }

private:
};

#endif
