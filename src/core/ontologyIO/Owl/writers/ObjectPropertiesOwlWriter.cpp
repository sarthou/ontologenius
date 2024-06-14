#include "ontologenius/core/ontologyIO/Owl/writers/ObjectPropertiesOwlWriter.h"

#include <cstdio>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

  ObjectPropertiesOwlWriter::ObjectPropertiesOwlWriter(ObjectPropertyGraph* property_graph,
                                                       const std::string& ns) : property_graph_(property_graph)
  {
    ns_ = ns;
  }

  void ObjectPropertiesOwlWriter::write(FILE* file)
  {
    file_ = file;

    const std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

    const std::vector<ObjectPropertyBranch*> properties = property_graph_->get();
    for(auto* property : properties)
      writeProperty(property);

    file_ = nullptr;
  }

  void ObjectPropertiesOwlWriter::writeProperty(ObjectPropertyBranch* branch)
  {
    std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:ObjectProperty rdf:about=\"" +
                      ns_ + "#" + branch->value() + "\">\n";
    writeString(tmp);

    writeSubPropertyOf(branch);
    writeDisjointWith(branch);
    writeInverseOf(branch);
    writeProperties(branch);
    writeRange(branch);
    writeDomain(branch);
    writeChain(branch);

    writeDictionary(branch);
    writeMutedDictionary(branch);

    tmp = "    </owl:ObjectProperty>\n\n\n\n";
    writeString(tmp);
  }

  void ObjectPropertiesOwlWriter::writeSubPropertyOf(ObjectPropertyBranch* branch)
  {
    for(auto& mother : branch->mothers_)
      if(mother.infered == false)
      {
        const std::string tmp = "        <rdfs:subPropertyOf" +
                                getProba(mother) +
                                " rdf:resource=\"" + ns_ + "#" +
                                mother.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void ObjectPropertiesOwlWriter::writeInverseOf(ObjectPropertyBranch* branch)
  {
    for(auto& inverse : branch->inverses_)
      if(inverse.infered == false)
      {
        const std::string tmp = "        <owl:inverseOf" +
                                getProba(inverse) +
                                " rdf:resource=\"" + ns_ + "#" +
                                inverse.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void ObjectPropertiesOwlWriter::writeRange(ObjectPropertyBranch* branch)
  {
    for(auto& range : branch->ranges_)
      if(range.infered == false)
      {
        const std::string tmp = "        <rdfs:range" +
                                getProba(range) +
                                " rdf:resource=\"" + ns_ + "#" +
                                range.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void ObjectPropertiesOwlWriter::writeDomain(ObjectPropertyBranch* branch)
  {
    for(auto& domain : branch->domains_)
      if(domain.infered == false)
      {
        const std::string tmp = "        <rdfs:domain" +
                                getProba(domain) +
                                " rdf:resource=\"" + ns_ + "#" +
                                domain.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void ObjectPropertiesOwlWriter::writeChain(ObjectPropertyBranch* branch)
  {
    for(auto& chain : branch->str_chains_)
    {
      std::string tmp = "        <owl:propertyChainAxiom rdf:parseType=\"Collection\">\n";

      for(auto& link : chain)
      {
        tmp += "            <rdf:Description rdf:about=\"" + ns_ + "#" +
               link +
               "\"/>\n";
      }

      tmp += "        </owl:propertyChainAxiom>\n";
      writeString(tmp);
    }
  }

} // namespace ontologenius
