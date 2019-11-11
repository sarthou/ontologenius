#include "ontoloGenius/core/ontologyIO/writers/DataPropertiesWriter.h"

#include <vector>

#include "ontoloGenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

namespace ontologenius {

void DataPropertiesWriter::write(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

  std::vector<DataPropertyBranch_t*> properties = property_graph_->get();
  for(size_t i = 0; i < properties.size(); i++)
    writeProperty(properties[i]);

  file_ = nullptr;
}

void DataPropertiesWriter::writeProperty(DataPropertyBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value() + " -->\n\n\
    <owl:DatatypeProperty rdf:about=\"ontologenius#" + branch->value() + "\">\n";
  writeString(tmp);

  writeSubPropertyOf(branch);
  writeDisjointWith(&branch->steady_);
  writeProperties(&branch->steady_);
  writeRange(branch);
  writeDomain(branch);

  writeDictionary(&branch->steady_);
  writeMutedDictionary(&branch->steady_);

  tmp = "    </owl:DatatypeProperty>\n\n\n\n";
  writeString(tmp);
}

void DataPropertiesWriter::writeSubPropertyOf(DataPropertyBranch_t* branch)
{
  for(auto& mother : branch->steady_.mothers_)
  {
    std::string proba = (mother < 1.0) ? " onto:probability=\"" + std::to_string(mother.probability) + "\"" : "";
    std::string tmp = "        <rdfs:subPropertyOf" +
                      proba +
                      " rdf:resource=\"ontologenius#" +
                      mother.elem->value()
                      + "\"/>\n";
    writeString(tmp);
  }
}

void DataPropertiesWriter::writeRange(DataPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.ranges_.size(); i++)
  {
    std::string tmp = "        <rdfs:range rdf:resource=\"" +
                      branch->steady_.ranges_[i].getNs() +
                      "#" +
                      branch->steady_.ranges_[i].type_ +
                      + "\"/>\n";
    writeString(tmp);
  }
}

void DataPropertiesWriter::writeDomain(DataPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.domains_.size(); i++)
  {
    std::string tmp = "        <rdfs:domain rdf:resource=\"ontologenius#" +
                      branch->steady_.domains_[i]->value()
                      + "\"/>\n";
    writeString(tmp);
  }
}

} // namespace ontologenius
