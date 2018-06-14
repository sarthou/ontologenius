#include "ontoloGenius/core/ontoGraphs/writers/ClassWriter.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"

#include <vector>

void ClassWriter::write(FILE* file)
{
  file_ = file;

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  for(size_t i = 0; i < classes.size(); i++)
    writeClass(classes[i]);

  file_ = nullptr;
}

void ClassWriter::writeClass(ClassBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value_ + " -->\n\r\n\r\
    <owl:Class rdf:about=\"ontologenius#" + branch->value_ + "\">\n\r";
  writeString(tmp);

  tmp = "    </owl:Class>\n\r\n\r\n\r\n\r";
  writeString(tmp);
}

void ClassWriter::writeString(std::string text)
{
  if(file_ != NULL)
    fwrite(text.c_str(), sizeof(char), text.size(), file_);
}
