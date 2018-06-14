#include "ontoloGenius/core/ontoGraphs/writers/ObjectPropertiesWriter.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

#include <vector>

void ObjectPropertiesWriter::write(FILE* file)
{
  file_ = file;

  std::vector<ObjectPropertyBranch_t*> properties = property_graph_->get();
  for(size_t i = 0; i < properties.size(); i++)
    writeProperty(properties[i]);

  file_ = nullptr;
}

void ObjectPropertiesWriter::writeProperty(ObjectPropertyBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value_ + " -->\n\r\n\r\
    <owl:ObjectProperty rdf:about=\"ontologenius#" + branch->value_ + "\">\n\r";
  writeString(tmp);

  tmp = "    </owl:ObjectProperty>\n\r\n\r\n\r\n\r";
  writeString(tmp);
}

void ObjectPropertiesWriter::writeString(std::string text)
{
  if(file_ != NULL)
    fwrite(text.c_str(), sizeof(char), text.size(), file_);
}
