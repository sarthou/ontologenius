#include "ontoloGenius/core/ontologyIO/writers/ObjectPropertiesWriter.h"

#include <vector>

#include "ontoloGenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"

namespace ontologenius {

void ObjectPropertiesWriter::write(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(property_graph_->mutex_);

  std::vector<ObjectPropertyBranch_t*> properties = property_graph_->get();
  for(size_t i = 0; i < properties.size(); i++)
    writeProperty(properties[i]);

  file_ = nullptr;
}

void ObjectPropertiesWriter::writeProperty(ObjectPropertyBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value() + " -->\n\n\
    <owl:ObjectProperty rdf:about=\"ontologenius#" + branch->value() + "\">\n";
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

void ObjectPropertiesWriter::writeSubPropertyOf(ObjectPropertyBranch_t* branch)
{
  for(auto& mother : branch->mothers_)
    if(mother.infered == false)
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

void ObjectPropertiesWriter::writeInverseOf(ObjectPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->inverses_.size(); i++)
  {
    std::string tmp = "        <owl:inverseOf rdf:resource=\"ontologenius#" +
                      branch->inverses_[i]->value()
                      + "\"/>\n";
    writeString(tmp);
  }
}

void ObjectPropertiesWriter::writeRange(ObjectPropertyBranch_t* branch)
{
  for(auto& range : branch->ranges_)
    if(range.infered == false)
    {
      std::string tmp = "        <rdfs:range" +
                        getProba(range) +
                        " rdf:resource=\"ontologenius#" +
                        range.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void ObjectPropertiesWriter::writeDomain(ObjectPropertyBranch_t* branch)
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

void ObjectPropertiesWriter::writeChain(ObjectPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->str_chains_.size(); i++)
  {
    std::string tmp = "        <owl:propertyChainAxiom rdf:parseType=\"Collection\">\n";

    for(size_t j = 0; j < branch->str_chains_[i].size(); j++)
    {
      tmp += "            <rdf:Description rdf:about=\"ontologenius#" +
              branch->str_chains_[i][j] +
              "\"/>\n";
    }

    tmp += "        </owl:propertyChainAxiom>\n";
    writeString(tmp);
  }
}

} // namespace ontologenius
