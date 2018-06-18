#include "ontoloGenius/core/ontoGraphs/writers/IndividualWriter.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include <vector>

void IndividualWriter::write(FILE* file)
{
  file_ = file;

  std::vector<IndividualBranch_t*> individuals = individual_graph_->get();
  for(size_t i = 0; i < individuals.size(); i++)
    writeIndividual(individuals[i]);

  file_ = nullptr;
}

void IndividualWriter::writeIndividual(IndividualBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value_ + " -->\n\r\n\r\
    <owl:NamedIndividual rdf:about=\"ontologenius#" + branch->value_ + "\">\n\r";
  writeString(tmp);

  writeType(branch);
  writeObjectProperties(branch);
  writeDataProperties(branch);

  writeDictionary(&branch->steady_);

  tmp = "    </owl:NamedIndividual>\n\r\n\r\n\r\n\r";
  writeString(tmp);
}

void IndividualWriter::writeType(IndividualBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.is_a_.size(); i++)
  {
    std::string tmp = "        <rdf:type rdf:resource=\"ontologenius#" +
                      branch->steady_.is_a_[i]->value_
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

void IndividualWriter::writeObjectProperties(IndividualBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.object_properties_name_.size(); i++)
  {
    std::string tmp = "        <ontologenius:" +
                      branch->steady_.object_properties_name_[i]->value_ +
                      " rdf:resource=\"ontologenius#" +
                      branch->steady_.object_properties_on_[i]->value_ +
                      "\"/>\n\r";
    writeString(tmp);
  }
}

void IndividualWriter::writeDataProperties(IndividualBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.data_properties_name_.size(); i++)
  {
    std::string tmp = "        <ontologenius:" +
                      branch->steady_.data_properties_name_[i]->value_ +
                      " rdf:datatype=\"" +
                      branch->steady_.data_properties_data_[i].getNs() +
                      "#" +
                      branch->steady_.data_properties_data_[i].type_ +
                      "\">" +
                      branch->steady_.data_properties_data_[i].value_ +
                      "</ontologenius:" +
                      branch->steady_.data_properties_name_[i]->value_ +
                      ">\n\r";
    writeString(tmp);
  }
}

void IndividualWriter::writeSameAs(IndividualBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.same_as_.size(); i++)
  {
    std::string tmp = "        <owl:sameAs rdf:resource=\"ontologenius#" +
                      branch->steady_.same_as_[i]->value_
                      + "\"/>\n\r";
    writeString(tmp);
  }
}
