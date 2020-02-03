#ifndef ONTOLOGENIUS_FILEWRITER_H
#define ONTOLOGENIUS_FILEWRITER_H

#include <string>

#include "ontologenius/interpreter/fileManagers/FileManager.h"

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

#endif // ONTOLOGENIUS_FILEWRITER_H
