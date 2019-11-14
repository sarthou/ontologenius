#include "ontoloGenius/core/ontologyIO/writers/ClassWriter.h"

#include <algorithm>

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"


namespace ontologenius {

void ClassWriter::write(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  for(size_t i = 0; i < classes.size(); i++)
    writeClass(classes[i]);

  file_ = nullptr;
}

void ClassWriter::writeGeneralAxioms(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  writeDisjointWith(classes);

  file_ = nullptr;
}


void ClassWriter::writeClass(ClassBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value() + " -->\n\n\
    <owl:Class rdf:about=\"ontologenius#" + branch->value() + "\">\n";
  writeString(tmp);

  writeSubClassOf(branch);

  writeDisjointWith(branch);

  writeObjectProperties(branch);
  writeDataProperties(branch);

  writeDictionary(branch);
  writeMutedDictionary(branch);

  tmp = "    </owl:Class>\n\n\n\n";
  writeString(tmp);
}

void ClassWriter::writeSubClassOf(ClassBranch_t* branch)
{
  for(auto& mother : branch->mothers_)
    if(mother.infered == false)
    {
      std::string proba = (mother < 1.0) ? " onto:probability=\"" + std::to_string(mother.probability) + "\"" : "";
      std::string tmp = "        <rdfs:subClassOf" +
                        proba +
                        " rdf:resource=\"ontologenius#" +
                        mother.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void ClassWriter::writeDisjointWith(ClassBranch_t* branch)
{
  if(branch->disjoints_.size() < 2)
    for(size_t i = 0; i < branch->disjoints_.size(); i++)
    {
      std::string tmp = "        <owl:disjointWith" +
                        getProba(branch->disjoints_[i]) +
                        " rdf:resource=\"ontologenius#" +
                        branch->disjoints_[i].elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void ClassWriter::writeDisjointWith(std::vector<ClassBranch_t*>& classes)
{
  std::vector<std::string> disjoint_done;

  std::string start = "    <rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDisjointClasses\"/>\n";

  std::string end = "    </rdf:Description>\n";

  for(size_t i = 0; i < classes.size(); i++)
  {
    if(classes[i]->disjoints_.size() > 1)
    {
      if(std::find(disjoint_done.begin(), disjoint_done.end(), classes[i]->value()) == disjoint_done.end())
      {
        std::string tmp;
        std::vector<std::string> disjoints_current;

        for(size_t j = 0; j < classes[i]->disjoints_.size(); j++)
          disjoints_current.push_back(classes[i]->disjoints_[j].elem->value());
        disjoints_current.push_back(classes[i]->value());

        getDisjoints(classes[i], disjoints_current);

        tmp += "        <owl:members rdf:parseType=\"Collection\">\n";

        for(size_t j = 0; j < disjoints_current.size(); j++)
        {
          disjoint_done.push_back(disjoints_current[j]);
          tmp += "             <rdf:Description rdf:about=\"ontologenius#" +
          disjoints_current[j] +
          "\"/>\n";
        }

        tmp += "        </owl:members>\n";
        if(disjoints_current.size() != 0)
        {
          writeString(start);
          writeString(tmp);
          writeString(end);
        }
      }
    }
  }
}

void ClassWriter::getDisjoints(ClassBranch_t* class_branch, std::vector<std::string>& disjoints_current)
{
  for(size_t i = 0; i < class_branch->disjoints_.size(); i++)
  {
    std::vector<std::string> disjoints_class;
    for(size_t j = 0; j < class_branch->disjoints_[i].elem->disjoints_.size(); j++)
      disjoints_class.push_back(class_branch->disjoints_[i].elem->disjoints_[j].elem->value());
    disjoints_class.push_back(class_branch->disjoints_[i].elem->value());
    removeDifferents(disjoints_current, disjoints_class);
  }
}

void ClassWriter::removeDifferents(std::vector<std::string>& disjoints_current, std::vector<std::string>& disjoints_class)
{
  std::vector<std::string> sames;
  for(size_t word = 0; word < disjoints_current.size(); word++)
  {
    std::vector<std::string>::iterator it = find(disjoints_class.begin(), disjoints_class.end(), disjoints_current[word]);
    if(it != disjoints_class.end())
      sames.push_back(disjoints_current[word]);
  }

  disjoints_current = sames;
}

void ClassWriter::writeObjectProperties(ClassBranch_t* branch)
{
  for(ClassObjectRelationElement_t& relation : branch->object_relations_)
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

void ClassWriter::writeDataProperties(ClassBranch_t* branch)
{
  for(ClassDataRelationElement_t& relation : branch->data_relations_)
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

} // namespace ontologenius
