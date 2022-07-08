#ifndef ONTOLOGENIUS_ONTOLOGYREADER_H
#define ONTOLOGENIUS_ONTOLOGYREADER_H

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"

namespace ontologenius {

class Ontology;

class OntologyReader
{
public:
  OntologyReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph);
  explicit OntologyReader(Ontology& onto);
  ~OntologyReader() {}

  void setDisplay(bool display) { display_ = display; }
  bool empty() {return (elem_loaded == 0); }

protected:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;
  IndividualGraph* individual_graph_;

  int elem_loaded;
  bool display_;

  void push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole)
  {
    vect.push_back(elem);
    if(symbole != "" && display_)
      std::cout << "│   │   ├── " << symbole << elem << std::endl;
  }

  void push(std::vector<bool>& vect, bool elem, const std::string& symbole)
  {
    vect.push_back(elem);
    if(symbole != "" && display_)
    {
      if(elem == true)
        std::cout << "│   │   ├── " << symbole << " true" << std::endl;
      else
        std::cout << "│   │   ├── " << symbole << " false" << std::endl;
    }
  }

  void push(std::vector<Pair_t<std::string, std::string>>& vect, const Pair_t<std::string, std::string>& elem, const std::string& symbole1, const std::string& symbole2)
  {
    vect.push_back(elem);
    if(symbole1 != "" && display_)
      std::cout << "│   │   ├── " << symbole1 << elem.first << std::endl;

    if(symbole2 != "" && display_)
      std::cout << "│   │   ├── " << symbole2 << elem.second << std::endl;
  }

  void push(std::vector<Pair_t<std::string, data_t>>& vect, const Pair_t<std::string, data_t>& elem, const std::string& symbole1, const std::string& symbole2)
  {
    vect.push_back(elem);
    if(symbole1 != "" && display_)
      std::cout << "│   │   ├── " << symbole1 << elem.first << std::endl;

    if(symbole2 != "" && display_)
      std::cout << "│   │   ├── " << symbole2 << elem.second.toString() << std::endl;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYREADER_H