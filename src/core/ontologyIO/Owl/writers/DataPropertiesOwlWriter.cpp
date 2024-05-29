#include "ontologenius/core/ontologyIO/Owl/writers/DataPropertiesOwlWriter.h"

#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

namespace ontologenius {

  DataPropertiesOwlWriter::DataPropertiesOwlWriter(DataPropertyGraph* property_graph, const std::string& ns)
  {
    property_graph_ = property_graph;
    ns_ = ns;
  }

  void DataPropertiesOwlWriter::write(FILE* file)
  {
    file_ = file;

    std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

    std::vector<DataPropertyBranch*> properties = property_graph_->get();
    for(auto property : properties)
      writeProperty(property);

    file_ = nullptr;
  }

  void DataPropertiesOwlWriter::writeProperty(DataPropertyBranch* branch)
  {
    std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:DatatypeProperty rdf:about=\"" +
                      ns_ + "#" + branch->value() + "\">\n";
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

  void DataPropertiesOwlWriter::writeSubPropertyOf(DataPropertyBranch* branch)
  {
    for(auto& mother : branch->mothers_)
      if(mother.infered == false)
      {
        std::string tmp = "        <rdfs:subPropertyOf" +
                          getProba(mother) +
                          " rdf:resource=\"" + ns_ + "#" +
                          mother.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void DataPropertiesOwlWriter::writeRange(DataPropertyBranch* branch)
  {
    for(auto& range : branch->ranges_)
    {
      std::string tmp = "        <rdfs:range rdf:resource=\"" +
                        range->getNs() +
                        "#" +
                        range->type_ +
                        +"\"/>\n";
      writeString(tmp);
    }
  }

  void DataPropertiesOwlWriter::writeDomain(DataPropertyBranch* branch)
  {
    for(auto& domain : branch->domains_)
      if(domain.infered == false)
      {
        std::string tmp = "        <rdfs:domain" +
                          getProba(domain) +
                          " rdf:resource=\"" + ns_ + "#" +
                          domain.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

} // namespace ontologenius
