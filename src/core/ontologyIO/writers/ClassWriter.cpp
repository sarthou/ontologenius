#include "ontoloGenius/core/ontologyIO/writers/ClassWriter.h"

#include "ontoloGenius/core/ontoGraphs/Graphs/ClassGraph.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"
#include "ontoloGenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"

#include <algorithm>

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
  std::string tmp = "    <!-- ontologenius#" + branch->value() + " -->\n\r\n\r\
    <owl:Class rdf:about=\"ontologenius#" + branch->value() + "\">\n\r";
  writeString(tmp);

  writeSubClassOf(branch);

  writeDisjointWith(branch);

  writeObjectProperties(branch);
  writeObjectPropertiesDeduced(branch);

  writeDataProperties(branch);
  writeDataPropertiesDeduced(branch);

  writeDictionary(&branch->steady_);

  tmp = "    </owl:Class>\n\r\n\r\n\r\n\r";
  writeString(tmp);
}

void ClassWriter::writeSubClassOf(ClassBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.mothers_.size(); i++)
  {
    std::string tmp = "        <rdfs:subClassOf rdf:resource=\"ontologenius#" +
                      branch->steady_.mothers_[i]->value()
                      + "\"/>\n\r";
    writeString(tmp);
  }
}

void ClassWriter::writeDisjointWith(ClassBranch_t* branch)
{
  if(branch->steady_.disjoints_.size() < 2)
    for(size_t i = 0; i < branch->steady_.disjoints_.size(); i++)
    {
      std::string tmp = "        <owl:disjointWith rdf:resource=\"ontologenius#" +
                        branch->steady_.disjoints_[i]->value()
                        + "\"/>\n\r";
      writeString(tmp);
    }
}

void ClassWriter::writeDisjointWith(std::vector<ClassBranch_t*>& classes)
{
  std::vector<std::string> disjoint_done;

  std::string start = "    <rdf:Description>\n\r\
        <rdf:type rdf:resource=\"http://www.w3.org/2002/07/owl#AllDisjointClasses\"/>\n\r";

  std::string end = "    </rdf:Description>\n\r";

  for(size_t i = 0; i < classes.size(); i++)
  {
    if(classes[i]->disjoints_.size() > 1)
    {
      if(std::find(disjoint_done.begin(), disjoint_done.end(), classes[i]->value()) == disjoint_done.end())
      {
        std::string tmp;
        std::vector<std::string> disjoints_current;

        for(size_t j = 0; j < classes[i]->disjoints_.size(); j++)
          disjoints_current.push_back(classes[i]->disjoints_[j]->value());
        disjoints_current.push_back(classes[i]->value());

        getDisjoints(classes[i], disjoints_current);

        tmp += "        <owl:members rdf:parseType=\"Collection\">\n\r";

        for(size_t j = 0; j < disjoints_current.size(); j++)
        {
          disjoint_done.push_back(disjoints_current[j]);
          tmp += "             <rdf:Description rdf:about=\"ontologenius#" +
          disjoints_current[j] +
          "\"/>\n\r";
        }

        tmp += "        </owl:members>\n\r";
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
    for(size_t j = 0; j < class_branch->disjoints_[i]->disjoints_.size(); j++)
      disjoints_class.push_back(class_branch->disjoints_[i]->disjoints_[j]->value());
    disjoints_class.push_back(class_branch->disjoints_[i]->value());
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
  for(size_t i = 0; i < branch->steady_.object_properties_name_.size(); i++)
  {
    std::string tmp = "        <ontologenius:" +
                      branch->steady_.object_properties_name_[i]->value() +
                      " rdf:resource=\"ontologenius#" +
                      branch->steady_.object_properties_on_[i]->value() +
                      "\"/>\n\r";
    writeString(tmp);
  }
}

void ClassWriter::writeObjectPropertiesDeduced(ClassBranch_t* branch)
{
  for(size_t i = 0; i < branch->object_properties_name_.size(); i++)
    if(branch->object_properties_deduced_[i] == true)
    {
      std::string tmp = "        <ontologenius:" +
                        branch->object_properties_name_[i]->value() +
                        " rdf:resourceDeduced=\"ontologenius#" +
                        branch->object_properties_on_[i]->value() +
                        "\"/>\n\r";
      writeString(tmp);
    }
}

void ClassWriter::writeDataProperties(ClassBranch_t* branch)
{
  for(size_t i = 0; i < branch->steady_.data_properties_name_.size(); i++)
  {
    std::string tmp = "        <ontologenius:" +
                      branch->steady_.data_properties_name_[i]->value() +
                      " rdf:datatype=\"" +
                      branch->steady_.data_properties_data_[i].getNs() +
                      "#" +
                      branch->steady_.data_properties_data_[i].type_ +
                      "\">" +
                      branch->steady_.data_properties_data_[i].value_ +
                      "</ontologenius:" +
                      branch->steady_.data_properties_name_[i]->value() +
                      ">\n\r";
    writeString(tmp);
  }
}

void ClassWriter::writeDataPropertiesDeduced(ClassBranch_t* branch)
{
  for(size_t i = 0; i < branch->data_properties_name_.size(); i++)
    if(branch->data_properties_deduced_[i] == true)
    {
      std::string tmp = "        <ontologenius:" +
                        branch->data_properties_name_[i]->value() +
                        " rdf:datatypeDeduced=\"" +
                        branch->data_properties_data_[i].getNs() +
                        "#" +
                        branch->data_properties_data_[i].type_ +
                        "\">" +
                        branch->data_properties_data_[i].value_ +
                        "</ontologenius:" +
                        branch->data_properties_name_[i]->value() +
                        ">\n\r";
      writeString(tmp);
    }
}

} // namespace ontologenius
