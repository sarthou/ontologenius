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
