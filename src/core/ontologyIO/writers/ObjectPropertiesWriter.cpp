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
  std::string tmp = "    <!-- ontologenius#" + branch->value() + " -->\n\r\n\r\
    <owl:ObjectProperty rdf:about=\"ontologenius#" + branch->value() + "\">\n\r";
  writeString(tmp);

  writeSubPropertyOf(branch);
  writeDisjointWith(&branch->steady_);
  writeInverseOf(branch);
  writeProperties(&branch->steady_);
  writeRange(branch);
  writeDomain(branch);
  writeChain(branch);

  writeDictionary(&branch->steady_);
  writeMutedDictionary(&branch->steady_);

  tmp = "    </owl:ObjectProperty>\n\r\n\r\n\r\n\r";
  writeString(tmp);
}

void ObjectPropertiesWriter::writeSubPropertyOf(ObjectPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.mothers_.size(); i++)
  {
    std::string tmp = "        <rdfs:subPropertyOf rdf:resource=\"ontologenius#" +
                      branch->steady_.mothers_[i]->value()
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

void ObjectPropertiesWriter::writeInverseOf(ObjectPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.inverses_.size(); i++)
  {
    std::string tmp = "        <owl:inverseOf rdf:resource=\"ontologenius#" +
                      branch->steady_.inverses_[i]->value()
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

void ObjectPropertiesWriter::writeRange(ObjectPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.ranges_.size(); i++)
  {
    std::string tmp = "        <rdfs:range rdf:resource=\"ontologenius#" +
                      branch->steady_.ranges_[i]->value()
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

void ObjectPropertiesWriter::writeDomain(ObjectPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.domains_.size(); i++)
  {
    std::string tmp = "        <rdfs:domain rdf:resource=\"ontologenius#" +
                      branch->steady_.domains_[i]->value()
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

void ObjectPropertiesWriter::writeChain(ObjectPropertyBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.str_chains_.size(); i++)
  {
    std::string tmp = "        <owl:propertyChainAxiom rdf:parseType=\"Collection\">\n\r";

    for(size_t j = 0; j < branch->steady_.str_chains_[i].size(); j++)
    {
      tmp += "            <rdf:Description rdf:about=\"ontologenius#" +
              branch->steady_.str_chains_[i][j] +
              "\"/>\n\r";
    }

    tmp += "        </owl:propertyChainAxiom>\n\r";
    writeString(tmp);
  }
}

} // namespace ontologenius
