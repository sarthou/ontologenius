#include "ontologenius/core/ontologyIO/writers/IndividualWriter.h"

#include <algorithm>
#include <vector>

#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

void IndividualWriter::write(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

  std::vector<IndividualBranch_t*> individuals = individual_graph_->get();
  for(auto& individual : individuals)
    writeIndividual(individual);

  file_ = nullptr;
}

void IndividualWriter::writeGeneralAxioms(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

  std::vector<IndividualBranch_t*> individuals = individual_graph_->get();
  writeDistincts(individuals);

  file_ = nullptr;
}

void IndividualWriter::writeIndividual(IndividualBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value() + " -->\n\n\
    <owl:NamedIndividual rdf:about=\"ontologenius#" + branch->value() + "\">\n";
  writeString(tmp);

  writeType(branch);
  writeObjectProperties(branch);
  writeDataProperties(branch);
  writeSameAs(branch);

  writeDictionary(branch);
  writeMutedDictionary(branch);

  tmp = "    </owl:NamedIndividual>\n\n\n\n";
  writeString(tmp);
}

void IndividualWriter::writeType(IndividualBranch_t* branch)
{
  for(auto& mother : branch->is_a_)
    if(mother.infered == false)
    {
      std::string proba = (mother < 1.0) ? " onto:probability=\"" + std::to_string(mother.probability) + "\"" : "";
      std::string tmp = "        <rdf:type" +
                        proba +
                        " rdf:resource=\"ontologenius#" +
                        mother.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void IndividualWriter::writeObjectProperties(IndividualBranch_t* branch)
{
  for(IndivObjectRelationElement_t& relation : branch->object_relations_)
    if(relation.infered == false)
    {
      std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
      std::string tmp = "        <ontologenius:" +
                        relation.first->value() +
                        proba +
                        " rdf:resource=\"ontologenius#" +
                        relation.second->value() +
                        "\"/>\n";
      writeString(tmp);
    }
}

void IndividualWriter::writeDataProperties(IndividualBranch_t* branch)
{
  for(IndivDataRelationElement_t& relation : branch->data_relations_)
    if(relation.infered == false)
    {
      std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
      std::string tmp = "        <ontologenius:" +
                        relation.first->value() +
                        proba +
                        " rdf:datatype=\"" +
                        relation.second.getNs() +
                        "#" +
                        relation.second.type_ +
                        "\">" +
                        relation.second.value_ +
                        "</ontologenius:" +
                        relation.first->value() +
                        ">\n";
      writeString(tmp);
    }
}
void IndividualWriter::writeSameAs(IndividualBranch_t* branch)
{
  for(auto& same_as : branch->same_as_)
    if(same_as.infered == false)
    {
      std::string tmp = "        <owl:sameAs rdf:resource=\"ontologenius#" +
                        same_as.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void IndividualWriter::writeDistincts(std::vector<IndividualBranch_t*>& individuals)
{
  std::vector<std::string> distincts_done;

  std::string start = "    <rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDifferent\"/>\n";

  std::string end = "    </rdf:Description>\n";

  for(auto& individual : individuals)
  {
    if(individual->distinct_.size() != 0)
    {
      if(std::find(distincts_done.begin(), distincts_done.end(), individual->value()) == distincts_done.end())
      {
        std::string tmp;
        std::vector<std::string> distincts_current;
        getDistincts(individual, distincts_current);

        tmp += "        <owl:distinctMembers rdf:parseType=\"Collection\">\n";

        for(const auto& distinct : distincts_current)
        {
          distincts_done.push_back(distinct);
          tmp += "             <rdf:Description rdf:about=\"ontologenius#" +
          distinct +
          "\"/>\n";
        }

        tmp += "        </owl:distinctMembers>\n";
        writeString(start);
        writeString(tmp);
        writeString(end);
      }
    }
  }
}

void IndividualWriter::getDistincts(IndividualBranch_t* individual, std::vector<std::string>& distincts_current)
{
  if(std::find(distincts_current.begin(), distincts_current.end(), individual->value()) == distincts_current.end())
  {
    distincts_current.push_back(individual->value());
    for(auto& distinct : individual->distinct_)
      getDistincts(distinct.elem, distincts_current);
  }
}

} // namespace ontologenius
