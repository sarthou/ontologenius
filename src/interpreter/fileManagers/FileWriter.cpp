#include "ontoloGenius/interpreter/fileManagers/FileWriter.h"

namespace ontologenius {

void FileWriter::write(std::string text)
{
  if(file_ != NULL)
    fwrite(text.c_str(), sizeof(char), text.size(), file_);
}

} // namespace ontologenius
