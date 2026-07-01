#include "ontologenius/core/ontologyIO/Owl/writers/IndividualOwlWriter.h"

#include <algorithm>
#include <cstdio>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontologyIO/Owl/writers/GraphOwlWriter.h"

namespace ontologenius {

  IndividualOwlWriter::IndividualOwlWriter(IndividualGraph* individual_graph,
                                           FILE* file,
                                           const std::string& ns) : GraphOwlWriter(file, ns, "owl:NamedIndividual"),
                                                                    individual_graph_(individual_graph)
  {}

  void IndividualOwlWriter::write()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

    std::vector<IndividualBranch*> individuals = individual_graph_->get();
    std::sort(individuals.begin(), individuals.end(),
              [](const IndividualBranch* a, const IndividualBranch* b) {
                return a->value() < b->value();
              });
    for(auto* individual : individuals)
      writeIndividual(individual);
  }

  void IndividualOwlWriter::writeGeneralAxioms()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(individual_graph_->mutex_);

    std::vector<IndividualBranch*> individuals = individual_graph_->get();
    writeDistincts(individuals);
  }

  void IndividualOwlWriter::writeIndividual(IndividualBranch* branch)
  {
    writeBranchStart(branch->value());

    writeType(branch);
    writeObjectProperties(branch);
    writeDataProperties(branch);
    writeSameAs(branch);

    writeDictionary(branch);
    writeMutedDictionary(branch);

    writeCommentDictionary(branch);

    writeBranchEnd();
  }

  void IndividualOwlWriter::writeType(IndividualBranch* branch)
  {
    for(auto& mother : branch->is_a_)
      if(mother.inferred == false)
        writeString("<rdf:type" + getProba(mother) + " rdf:resource=\"" + ns_ + "#" + mother.elem->value() + "\"/>\n", 2);
  }

  void IndividualOwlWriter::writeObjectProperties(IndividualBranch* branch)
  {
    for(const IndivObjectRelationElement& relation : branch->object_relations_)
      if(relation.inferred == false)
        writeString("<onto:" + relation.first->value() + getProba(relation) + " rdf:resource=\"" + ns_ + "#" + relation.second->value() + "\"/>\n", 2);
  }

  void IndividualOwlWriter::writeDataProperties(IndividualBranch* branch)
  {
    for(const IndivDataRelationElement& relation : branch->data_relations_)
      if(relation.inferred == false)
      {
        const std::string tmp = "<onto:" + relation.first->value() +
                                getProba(relation) + " " + getRdfDatatype(relation.second->type_) + ">" +
                                relation.second->data() +
                                "</onto:" + relation.first->value() + ">\n";
        writeString(tmp, 2);
      }
  }
  void IndividualOwlWriter::writeSameAs(IndividualBranch* branch)
  {
    for(auto& same_as : branch->same_as_)
      if(same_as.inferred == false)
        writeSingleResource("owl:sameAs", same_as);
  }

  void IndividualOwlWriter::writeDistincts(std::vector<IndividualBranch*>& individuals)
  {
    std::vector<std::string> distincts_done;

    const std::string start = "<rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDifferent\"/>\n";

    const std::string end = "</rdf:Description>\n";

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
          writeString(start, 1);
          writeString(tmp, 1);
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
