#include "ontologenius/core/ontologyIO/Owl/writers/IndividualOwlWriter.h"

#include <algorithm>
#include <cstdio>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

  IndividualOwlWriter::IndividualOwlWriter(IndividualGraph* individual_graph,
                                           const std::string& ns) : individual_graph_(individual_graph)
  {
    ns_ = ns;
  }

  void IndividualOwlWriter::write(FILE* file)
  {
    file_ = file;

    const std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

    const std::vector<IndividualBranch*> individuals = individual_graph_->get();
    for(auto* individual : individuals)
      writeIndividual(individual);

    file_ = nullptr;
  }

  void IndividualOwlWriter::writeGeneralAxioms(FILE* file)
  {
    file_ = file;

    const std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

    std::vector<IndividualBranch*> individuals = individual_graph_->get();
    writeDistincts(individuals);

    file_ = nullptr;
  }

  void IndividualOwlWriter::writeIndividual(IndividualBranch* branch)
  {
    std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:NamedIndividual rdf:about=\"" +
                      ns_ + "#" + branch->value() + "\">\n";
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

  void IndividualOwlWriter::writeType(IndividualBranch* branch)
  {
    for(auto& mother : branch->is_a_)
      if(mother.infered == false)
      {
        const std::string proba = (mother < 1.0) ? " onto:probability=\"" + std::to_string(mother.probability) + "\"" : "";
        const std::string tmp = "        <rdf:type" +
                                proba +
                                " rdf:resource=\"" + ns_ + "#" +
                                mother.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void IndividualOwlWriter::writeObjectProperties(IndividualBranch* branch)
  {
    for(const IndivObjectRelationElement& relation : branch->object_relations_)
      if(relation.infered == false)
      {
        const std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
        const std::string tmp = "        <" +
                                relation.first->value() +
                                proba +
                                " rdf:resource=\"" + ns_ + "#" +
                                relation.second->value() +
                                "\"/>\n";
        writeString(tmp);
      }
  }

  void IndividualOwlWriter::writeDataProperties(IndividualBranch* branch)
  {
    for(const IndivDataRelationElement& relation : branch->data_relations_)
      if(relation.infered == false)
      {
        const std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
        const std::string tmp = "        <" +
                                relation.first->value() +
                                proba +
                                " rdf:datatype=\"" +
                                relation.second->getNs() +
                                "#" +
                                relation.second->type_ +
                                "\">" +
                                relation.second->value_ +
                                "</" +
                                relation.first->value() +
                                ">\n";
        writeString(tmp);
      }
  }
  void IndividualOwlWriter::writeSameAs(IndividualBranch* branch)
  {
    for(auto& same_as : branch->same_as_)
      if(same_as.infered == false)
      {
        const std::string tmp = "        <owl:sameAs rdf:resource=\"" + ns_ + "#" +
                                same_as.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void IndividualOwlWriter::writeDistincts(std::vector<IndividualBranch*>& individuals)
  {
    std::vector<std::string> distincts_done;

    const std::string start = "    <rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDifferent\"/>\n";

    const std::string end = "    </rdf:Description>\n";

    for(auto& individual : individuals)
    {
      if(individual->distinct_.empty() == false)
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
            tmp += "             <rdf:Description rdf:about=\"" + ns_ + "#" +
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

  void IndividualOwlWriter::getDistincts(IndividualBranch* individual, std::vector<std::string>& distincts_current)
  {
    if(std::find(distincts_current.begin(), distincts_current.end(), individual->value()) == distincts_current.end())
    {
      distincts_current.push_back(individual->value());
      for(auto& distinct : individual->distinct_)
        getDistincts(distinct.elem, distincts_current);
    }
  }

} // namespace ontologenius
