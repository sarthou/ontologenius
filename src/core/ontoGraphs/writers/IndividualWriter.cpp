#include "ontoloGenius/core/ontoGraphs/writers/IndividualWriter.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/IndividualGraph.h"

#include <vector>
#include <algorithm>

void IndividualWriter::write(FILE* file)
{
  file_ = file;

  std::vector<IndividualBranch_t*> individuals = individual_graph_->get();
  for(size_t i = 0; i < individuals.size(); i++)
    writeIndividual(individuals[i]);

  file_ = nullptr;
}

void IndividualWriter::writeGeneralAxioms(FILE* file)
{
  file_ = file;

  std::vector<IndividualBranch_t*> individuals = individual_graph_->get();
  writeDistincts(individuals);

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

void IndividualWriter::writeDistincts(std::vector<IndividualBranch_t*>& individuals)
{
  std::vector<std::string> distincts_done;

  std::string start = "    <rdf:Description>\n\r\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDifferent\"/>\n\r";

  std::string end = "    </rdf:Description>\n\r";
  std::string tmp;

  for(size_t i = 0; i < individuals.size(); i++)
  {
    if(individuals[i]->distinct_.size() != 0)
    {
      if(std::find(distincts_done.begin(), distincts_done.end(), individuals[i]->value_) == distincts_done.end())
      {
        std::vector<std::string> distincts_current;
        getDistincts(individuals[i], distincts_current);

        tmp += "        <owl:distinctMembers rdf:parseType=\"Collection\">\n\r";

        for(size_t j = 0; j < distincts_current.size(); j++)
        {
          distincts_done.push_back(distincts_current[j]);
          tmp += "             <rdf:Description rdf:about=\"ontologenius#" +
          distincts_current[j] +
          "\"/>\n\r";
        }

        tmp += "        </owl:distinctMembers>\n\r";
        writeString(start);
        writeString(tmp);
        writeString(end);
      }
    }
  }
}

void IndividualWriter::getDistincts(IndividualBranch_t* individual, std::vector<std::string>& distincts_current)
{
  if(std::find(distincts_current.begin(), distincts_current.end(), individual->value_) == distincts_current.end())
  {
    distincts_current.push_back(individual->value_);
    for(size_t i = 0; i < individual->distinct_.size(); i++)
      getDistincts(individual->distinct_[i], distincts_current);
  }
}
