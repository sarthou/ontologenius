#include "ontoloGenius/core/ontoGraphs/writers/DataPropertiesWriter.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

#include <vector>

void DataPropertiesWriter::write(FILE* file)
{
  file_ = file;

  std::vector<DataPropertyBranch_t*> properties = property_graph_->get();
  for(size_t i = 0; i < properties.size(); i++)
    writeProperty(properties[i]);

  file_ = nullptr;
}

void DataPropertiesWriter::writeProperty(DataPropertyBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value_ + " -->\n\r\n\r\
    <owl:DatatypeProperty rdf:about=\"ontologenius#" + branch->value_ + "\">\n\r";
  writeString(tmp);

  writeSubPropertyOf(branch);

  writeDictionary(&branch->steady_);

  tmp = "    </owl:DatatypeProperty>\n\r\n\r\n\r\n\r";
  writeString(tmp);
}

void DataPropertiesWriter::writeSubPropertyOf(DataPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.mothers_.size(); i++)
  {
    std::string tmp = "        <rdfs:subPropertyOf rdf:resource=\"ontologenius#" +
                      branch->steady_.mothers_[i]->value_
                      + "\"/>\n\r";
    writeString(tmp);
  }
}
