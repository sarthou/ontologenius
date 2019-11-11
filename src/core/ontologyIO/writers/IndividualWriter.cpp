#include "ontoloGenius/core/ontologyIO/writers/IndividualWriter.h"

#include <vector>
#include <algorithm>

#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

void IndividualWriter::write(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

  std::vector<IndividualBranch_t*> individuals = individual_graph_->get();
  for(size_t i = 0; i < individuals.size(); i++)
    writeIndividual(individuals[i]);

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
  writeObjectPropertiesDeduced(branch);
  writeDataProperties(branch);
  writeDataPropertiesDeduced(branch);
  writeSameAs(branch);

  writeDictionary(&branch->steady_);
  writeMutedDictionary(&branch->steady_);

  tmp = "    </owl:NamedIndividual>\n\n\n\n";
  writeString(tmp);
}

void IndividualWriter::writeType(IndividualBranch_t* branch)
{
  for(auto& mother : branch->steady_.is_a_)
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
  for(IndivObjectRelationElement_t& relation : branch->steady_.object_relations_)
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

void IndividualWriter::writeObjectPropertiesDeduced(IndividualBranch_t* branch)
{
  for(IndivObjectRelationElement_t& relation : branch->object_relations_)
    if(relation < 1.0) // deduced 0.5
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
  for(IndivDataRelationElement_t& relation : branch->steady_.data_relations_)
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

void IndividualWriter::writeDataPropertiesDeduced(IndividualBranch_t* branch)
{
  for(IndivDataRelationElement_t& relation : branch->data_relations_)
    if(relation < 1.0) // deduced = 0.5
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
  for(size_t i = 0; i < branch->steady_.same_as_.size(); i++)
  {
    std::string tmp = "        <owl:sameAs rdf:resource=\"ontologenius#" +
                      branch->steady_.same_as_[i]->value()
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

  for(size_t i = 0; i < individuals.size(); i++)
  {
    if(individuals[i]->distinct_.size() != 0)
    {
      if(std::find(distincts_done.begin(), distincts_done.end(), individuals[i]->value()) == distincts_done.end())
      {
        std::string tmp;
        std::vector<std::string> distincts_current;
        getDistincts(individuals[i], distincts_current);

        tmp += "        <owl:distinctMembers rdf:parseType=\"Collection\">\n";

        for(size_t j = 0; j < distincts_current.size(); j++)
        {
          distincts_done.push_back(distincts_current[j]);
          tmp += "             <rdf:Description rdf:about=\"ontologenius#" +
          distincts_current[j] +
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
    for(size_t i = 0; i < individual->distinct_.size(); i++)
      getDistincts(individual->distinct_[i], distincts_current);
  }
}

} // namespace ontologenius
