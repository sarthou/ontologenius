#include "ontologenius/core/ontologyIO/Owl/writers/ClassOwlWriter.h"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <set>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontologyIO/Owl/writers/AnonymousClassOwlWriter.h"

namespace ontologenius {

  ClassOwlWriter::ClassOwlWriter(ClassGraph* class_graph,
                                 FILE* file,
                                 const std::string& ns) : AnonymousClassOwlWriter("owl:Class", file, ns),
                                                          class_graph_(class_graph)
  {}

  void ClassOwlWriter::write()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    std::vector<ClassBranch*> classes = class_graph_->get();
    std::sort(classes.begin(), classes.end(),
              [](const ClassBranch* a, const ClassBranch* b) {
                return a->value() < b->value();
              });

    for(auto* classe : classes)
      writeClass(classe);
  }

  void ClassOwlWriter::writeGeneralAxioms()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);

    std::vector<ClassBranch*> classes = class_graph_->get();
    writeDisjointWith(classes);
  }

  void ClassOwlWriter::writeClass(ClassBranch* branch)
  {
    writeBranchStart(branch->value());

    writeEquivalentClass(branch);
    writeSubClassOf(branch);

    writeDisjointWith(branch);

    writeObjectProperties(branch);
    writeDataProperties(branch);

    writeDictionary(branch);
    writeMutedDictionary(branch);

    writeCommentDictionary(branch);

    writeBranchEnd();
  }

  void ClassOwlWriter::writeEquivalentClass(ClassBranch* branch)
  {
    const AnonymousClassBranch* equiv = branch->equiv_anonymous_class_;

    if(equiv != nullptr)
    {
      for(auto* tree : equiv->ano_trees_)
      {
        std::string field = "owl:equivalentClass";
        const size_t level = 2;

        auto* tree_root_node = tree->root_node_;

        // single expression
        if(tree_root_node->sub_elements_.empty() &&
           tree_root_node->class_involved_ != nullptr &&
           tree_root_node->object_property_involved_ == nullptr)
        {
          writeString("<" + field + " " + getRdfResource(tree_root_node->class_involved_->value()) + "/>\n", level);
        }
        else // Collection of expressions
        {
          writeString("<" + field + ">\n", level);
          writeClassExpression(tree_root_node, level + 1);
          writeString("</" + field + ">\n", level);
        }
      }
    }
  }

  void ClassOwlWriter::writeSubClassOf(ClassBranch* branch)
  {
    for(auto& mother : branch->mothers_)
      writeSingleResource("rdfs:subClassOf", mother);

    // write complex subClassOf expressions (rdfs:subClassOf with anonymous restriction)
    const AnonymousClassBranch* sub = branch->sub_anonymous_class_;
    if(sub != nullptr)
    {
      for(auto* tree : sub->ano_trees_)
      {
        const size_t level = 2;
        auto* tree_root_node = tree->root_node_;

        // single named-class expression: <rdfs:subClassOf rdf:resource="..."/>
        if(tree_root_node->sub_elements_.empty() &&
           tree_root_node->class_involved_ != nullptr &&
           tree_root_node->object_property_involved_ == nullptr)
        {
          writeString("<rdfs:subClassOf " + getRdfResource(tree_root_node->class_involved_->value()) + "/>\n", level);
        }
        else
        {
          writeString("<rdfs:subClassOf>\n", level);
          writeClassExpression(tree_root_node, level + 1);
          writeString("</rdfs:subClassOf>\n", level);
        }
      }
    }
  }

  void ClassOwlWriter::writeDisjointWith(ClassBranch* branch)
  {
    if(branch->disjoints_.size() < 2)
      for(auto& disjoint : branch->disjoints_)
        writeSingleResource("owl:disjointWith", disjoint);
  }

  void ClassOwlWriter::writeDisjointWith(std::vector<ClassBranch*>& classes)
  {
    const std::string start = "    <rdf:Description>\n\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDisjointClasses\"/>\n";

    const std::string end = "    </rdf:Description>\n";

    std::set<std::set<ClassBranch*>> disjoints_vects;

    for(auto& classe : classes)
      if(classe->disjoints_.size() > 1)
        getDisjointsSets(classe, disjoints_vects);

    for(const auto& disjoints_set : disjoints_vects)
    {
      std::string tmp;
      tmp += "        <owl:members rdf:parseType=\"Collection\">\n";

      for(const auto& disj : disjoints_set)
      {
        tmp += "             <rdf:Description rdf:about=\"" + ns_ + "#" +
               disj->value() +
               "\"/>\n";
      }

      tmp += "        </owl:members>\n";
      if(disjoints_set.empty() == false)
      {
        writeString(start);
        writeString(tmp);
        writeString(end);
      }
    }
  }

  void ClassOwlWriter::getDisjointsSets(ClassBranch* base, std::set<std::set<ClassBranch*>>& res)
  {
    std::set<ClassBranch*> restriction_set;

    for(auto& disjoint : base->disjoints_)
      restriction_set.insert(disjoint.elem);
    restriction_set.insert(base);

    for(auto& disjoint : base->disjoints_)
    {
      std::set<ClassBranch*> base_set;
      base_set.insert(base);
      base_set.insert(disjoint.elem);
      getDisjointsSets(disjoint.elem, base_set, restriction_set, res);
    }
  }

  void ClassOwlWriter::getDisjointsSets(ClassBranch* last, const std::set<ClassBranch*>& base_set, const std::set<ClassBranch*>& restriction_set, std::set<std::set<ClassBranch*>>& res)
  {
    std::set<ClassBranch*> local_disjoints;
    for(auto& disjoint : last->disjoints_)
      local_disjoints.insert(disjoint.elem);
    std::vector<ClassBranch*> new_restriction_vect;
    std::set_intersection(restriction_set.begin(), restriction_set.end(), local_disjoints.begin(), local_disjoints.end(), std::back_inserter(new_restriction_vect));
    std::set<ClassBranch*> new_restriction_set;
    for(auto& it : new_restriction_vect)
      new_restriction_set.insert(it);

    bool leaf = true;
    for(auto& disjoint : last->disjoints_)
    {
      if(restriction_set.find(disjoint.elem) != restriction_set.end())
      {
        if(base_set.find(disjoint.elem) == base_set.end())
        {
          std::set<ClassBranch*> new_set = base_set;
          new_set.insert(disjoint.elem);
          getDisjointsSets(disjoint.elem, new_set, new_restriction_set, res);
          leaf = false;
        }
      }
    }

    if(leaf)
      res.insert(base_set);
  }

  void ClassOwlWriter::writeObjectProperties(ClassBranch* branch)
  {
    for(const ClassObjectRelationElement& relation : branch->object_relations_)
      if(relation.inferred == false)
        writeString("<" + relation.first->value() + getProba(relation) + " " + getRdfResource(relation.second->value()) + "/>\n", 2);
  }

  void ClassOwlWriter::writeDataProperties(ClassBranch* branch)
  {
    for(const ClassDataRelationElement& relation : branch->data_relations_)
      if(relation.inferred == false)
      {
        const std::string tmp = "<" + relation.first->value() + getProba(relation) + " " + getRdfDatatype(relation.second->type_) + ">" +
                                relation.second->data() + "</" + relation.first->value() + ">\n";
        writeString(tmp, 2);
      }
  }

} // namespace ontologenius
