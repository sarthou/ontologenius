#include "ontologenius/core/ontologyIO/Owl/writers/DataPropertiesOwlWriter.h"

#include <algorithm>
#include <cstdio>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontologyIO/Owl/writers/PropertiesOwlWriter.h"

namespace ontologenius {

  DataPropertiesOwlWriter::DataPropertiesOwlWriter(DataPropertyGraph* property_graph,
                                                   FILE* file,
                                                   const std::string& ns) : PropertiesOwlWriter(file, ns, "owl:DatatypeProperty"),
                                                                            property_graph_(property_graph)
  {}

  void DataPropertiesOwlWriter::write()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

    std::vector<DataPropertyBranch*> properties = property_graph_->get();
    std::sort(properties.begin(), properties.end(),
              [](const DataPropertyBranch* a, const DataPropertyBranch* b) {
                return a->value() < b->value();
              });

    for(auto* property : properties)
      writeProperty(property);
  }

  void DataPropertiesOwlWriter::writeProperty(DataPropertyBranch* branch)
  {
    writeBranchStart(branch->value());

    for(auto& mother : branch->mothers_)
      writeSingleResource("rdfs:subPropertyOf", mother);

    writeDisjointWith(branch);
    writeProperties(branch);

    for(auto& domain : branch->domains_)
      writeSingleResource("rdfs:domain", domain);
    writeRange(branch);

    writeDictionary(branch);
    writeMutedDictionary(branch);

    writeCommentDictionary(branch);

    writeBranchEnd();
  }

  void DataPropertiesOwlWriter::writeRange(DataPropertyBranch* branch)
  {
    for(auto& range : branch->ranges_)
      writeString("<rdfs:range rdf:resource=\"" + range->getNamespace() + "#" + range->value() + +"\"/>\n", 2);
  }

} // namespace ontologenius
