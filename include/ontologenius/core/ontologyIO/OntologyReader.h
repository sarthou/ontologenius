#ifndef ONTOLOGENIUS_ONTOLOGYREADER_H
#define ONTOLOGENIUS_ONTOLOGYREADER_H

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/ObjectPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/IndividualGraph.h"
#include "ontologenius/core/ontoGraphs/Graphs/AnonymousClassGraph.h"

namespace ontologenius {

class Ontology;

class OntologyReader
{
public:
  OntologyReader(ClassGraph* class_graph, ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, IndividualGraph* individual_graph, AnonymousClassGraph* anonymous_graph);
  explicit OntologyReader(Ontology& onto);
  ~OntologyReader() {}

  void setDisplay(bool display) { display_ = display; }
  int getNbLoadedElements() { return nb_loaded_elem_; }
  bool empty() {return (nb_loaded_elem_ == 0); }

protected:
  ClassGraph* class_graph_;
  ObjectPropertyGraph* object_property_graph_;
  DataPropertyGraph* data_property_graph_;
  IndividualGraph* individual_graph_;
  AnonymousClassGraph* anonymous_graph_;

  int nb_loaded_elem_;
  bool display_;

  void push(std::vector<std::string>& vect, const std::string& elem, const std::string& symbole)
  {
    vect.emplace_back(elem);
    if(display_ && symbole != "")
      std::cout << "│   │   ├── " << symbole << elem << std::endl;
  }

  void push(std::vector<bool>& vect, bool elem, const std::string& symbole)
  {
    vect.push_back(elem);
    if(display_ && symbole != "")
    {
      if(elem == true)
        std::cout << "│   │   ├── " << symbole << " true" << std::endl;
      else
        std::cout << "│   │   ├── " << symbole << " false" << std::endl;
    }
  }

  void push(std::vector<Pair_t<std::string, std::string>>& vect, const Pair_t<std::string, std::string>& elem, const std::string& symbole1, const std::string& symbole2)
  {
    vect.emplace_back(elem);
    if(display_)
    {
      if(symbole1 != "")
        std::cout << "│   │   ├── " << symbole1 << elem.first << std::endl;

      if(symbole2 != "")
        std::cout << "│   │   ├── " << symbole2 << elem.second << std::endl;
    }
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ONTOLOGYREADER_H