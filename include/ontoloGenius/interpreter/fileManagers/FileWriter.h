#ifndef FILEWRITER_H
#define FILEWRITER_H

#include "ontoloGenius/interpreter/fileManagers/FileManager.h"

#include <string>

namespace ontologenius {

class FileWriter : public FileManager
{
public:
  FileWriter() {}
  ~FileWriter() {}

  void write(std::string text);
  void writeLine(std::string text) { write(text + "\n"); }

private:
};

} // namespace ontologenius

#endif
