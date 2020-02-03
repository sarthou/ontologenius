#include "ontologenius/core/ontologyIO/writers/DataPropertiesWriter.h"

#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

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
  writeDisjointWith(branch);
  writeProperties(branch);
  writeRange(branch);
  writeDomain(branch);

  writeDictionary(branch);
  writeMutedDictionary(branch);

  tmp = "    </owl:DatatypeProperty>\n\n\n\n";
  writeString(tmp);
}

void DataPropertiesWriter::writeSubPropertyOf(DataPropertyBranch_t* branch)
{
  for(auto& mother : branch->mothers_)
    if(mother.infered == false)
    {
      std::string tmp = "        <rdfs:subPropertyOf" +
                        getProba(mother) +
                        " rdf:resource=\"ontologenius#" +
                        mother.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void DataPropertiesWriter::writeRange(DataPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->ranges_.size(); i++)
  {
    std::string tmp = "        <rdfs:range rdf:resource=\"" +
                      branch->ranges_[i].getNs() +
                      "#" +
                      branch->ranges_[i].type_ +
                      + "\"/>\n";
    writeString(tmp);
  }
}

void DataPropertiesWriter::writeDomain(DataPropertyBranch_t* branch)
{
  for(auto& domain : branch->domains_)
    if(domain.infered == false)
    {
      std::string tmp = "        <rdfs:domain" +
                        getProba(domain) +
                        " rdf:resource=\"ontologenius#" +
                        domain.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

} // namespace ontologenius
