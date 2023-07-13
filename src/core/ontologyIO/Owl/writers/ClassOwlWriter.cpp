#include "ontologenius/core/ontologyIO/Owl/writers/ClassOwlWriter.h"

#include <algorithm>

#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"

namespace ontologenius {

ClassOwlWriter::ClassOwlWriter(ClassGraph* class_graph, const std::string& ns)
{
  class_graph_ = class_graph;
  ns_ = ns;
}

void ClassOwlWriter::write(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  for(auto& classe : classes)
    writeClass(classe);

  file_ = nullptr;
}

void ClassOwlWriter::writeGeneralAxioms(FILE* file)
{
  file_ = file;

  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  writeDisjointWith(classes);

  file_ = nullptr;
}


void ClassOwlWriter::writeClass(ClassBranch_t* branch)
{
  std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:Class rdf:about=\"" + ns_ + "#" + branch->value() + "\">\n";
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

void ClassOwlWriter::writeSubClassOf(ClassBranch_t* branch)
{
  for(auto& mother : branch->mothers_)
    if(mother.infered == false)
    {
      std::string proba = (mother < 1.0) ? " onto:probability=\"" + std::to_string(mother.probability) + "\"" : "";
      std::string tmp = "        <rdfs:subClassOf" +
                        proba +
                        " rdf:resource=\"" + ns_ + "#" +
                        mother.elem->value()
                        + "\"/>\n";
      writeString(tmp);
    }
}

void ClassOwlWriter::writeDisjointWith(ClassBranch_t* branch)
{
  if(branch->disjoints_.size() < 2)
    for(auto& disjoint : branch->disjoints_)
      if(disjoint.infered == false)
      {
        std::string tmp = "        <owl:disjointWith" +
                          getProba(disjoint) +
                          " rdf:resource=\"" + ns_ + "#" +
                          disjoint.elem->value()
                          + "\"/>\n";
        writeString(tmp);
      }
}

void ClassOwlWriter::writeDisjointWith(std::vector<ClassBranch_t*>& classes)
{
  std::string start = "    <rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDisjointClasses\"/>\n";

  std::string end = "    </rdf:Description>\n";

  std::set<std::set<ClassBranch_t*>> disjoints_vects;

  for(auto& classe : classes)
  {
    if(classe->disjoints_.size() > 1)
      getDisjointsSets(classe, disjoints_vects);
  }

  for(auto& disjoints_set : disjoints_vects)
  {
    std::string tmp;
    tmp += "        <owl:members rdf:parseType=\"Collection\">\n";

    for(auto& disj : disjoints_set)
    {
      tmp += "             <rdf:Description rdf:about=\"" + ns_ + "#" +
      disj->value() +
      "\"/>\n";
    }

    tmp += "        </owl:members>\n";
    if(disjoints_set.size() > 0)
    {
      writeString(start);
      writeString(tmp);
      writeString(end);
    }
  }
}

void ClassOwlWriter::getDisjointsSets(ClassBranch_t* base, std::set<std::set<ClassBranch_t*>>& res)
{
  std::set<ClassBranch_t*> restriction_set;

  for(auto& disjoint : base->disjoints_)
    restriction_set.insert(disjoint.elem);
  restriction_set.insert(base);

  for(auto& disjoint : base->disjoints_)
  {
    std::set<ClassBranch_t*> base_set;
    base_set.insert(base);
    base_set.insert(disjoint.elem);
    getDisjointsSets(disjoint.elem, base_set, restriction_set, res);
  }
}

void ClassOwlWriter::getDisjointsSets(ClassBranch_t* last, const std::set<ClassBranch_t*>& base_set, const std::set<ClassBranch_t*>& restriction_set, std::set<std::set<ClassBranch_t*>>& res)
{
  std::set<ClassBranch_t*> local_disjoints;
  for(auto& disjoint : last->disjoints_)
    local_disjoints.insert(disjoint.elem);
  std::vector<ClassBranch_t*> new_restriction_vect;
  std::set_intersection(restriction_set.begin(), restriction_set.end(), local_disjoints.begin(), local_disjoints.end(), std::back_inserter(new_restriction_vect));
  std::set<ClassBranch_t*> new_restriction_set;
  for(auto& it : new_restriction_vect)
    new_restriction_set.insert(it);

  bool leaf = true;
  for(auto& disjoint : last->disjoints_)
  {
    if(restriction_set.find(disjoint.elem) != restriction_set.end())
    {
      if(base_set.find(disjoint.elem) == base_set.end())
      {
        std::set<ClassBranch_t*> new_set = base_set;
        new_set.insert(disjoint.elem);
        getDisjointsSets(disjoint.elem, new_set, new_restriction_set, res);
        leaf = false;
      }
    }
  }

  if(leaf)
    res.insert(base_set);
}

void ClassOwlWriter::writeObjectProperties(ClassBranch_t* branch)
{
  for(ClassObjectRelationElement_t& relation : branch->object_relations_)
    if(relation.infered == false)
    {
      std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
      std::string tmp = "        <" +
                        relation.first->value() +
                        proba +
                        " rdf:resource=\"" + ns_ + "#" +
                        relation.second->value() +
                        "\"/>\n";
      writeString(tmp);
    }
}

void ClassOwlWriter::writeDataProperties(ClassBranch_t* branch)
{
  for(ClassDataRelationElement_t& relation : branch->data_relations_)
    if(relation.infered == false)
    {
      std::string proba = (relation < 1.0) ? " onto:probability=\"" + std::to_string(relation.probability) + "\"" : "";
      std::string tmp = "        <" +
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

} // namespace ontologenius
