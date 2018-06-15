#include "ontoloGenius/core/ontoGraphs/writers/ClassWriter.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"

#include <vector>

void ClassWriter::write(FILE* file)
{
  file_ = file;

  std::vector<ClassBranch_t*> classes = class_graph_->get();
  for(size_t i = 0; i < classes.size(); i++)
    writeClass(classes[i]);

  file_ = nullptr;
}

void ClassWriter::writeClass(ClassBranch_t* branch)
{
  std::string tmp = "    <!-- ontologenius#" + branch->value_ + " -->\n\r\n\r\
    <owl:Class rdf:about=\"ontologenius#" + branch->value_ + "\">\n\r";
  writeString(tmp);

  writeSubClassOf(branch);

  writeDisjointWith(branch);

  writeDictionary(&branch->steady_);

  tmp = "    </owl:Class>\n\r\n\r\n\r\n\r";
  writeString(tmp);
}

void ClassWriter::writeSubClassOf(ClassBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.mothers_.size(); i++)
  {
    std::string tmp = "        <rdfs:subClassOf rdf:resource=\"ontologenius#" +
                      branch->steady_.mothers_[i]->value_
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

void ClassWriter::writeDisjointWith(ClassBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.disjoints_.size(); i++)
  {
    std::string tmp = "        <owl:disjointWith rdf:resource=\"ontologenius#" +
                      branch->steady_.disjoints_[i]->value_
                      + "\"/>\n\r";
    writeString(tmp);
  }
}
