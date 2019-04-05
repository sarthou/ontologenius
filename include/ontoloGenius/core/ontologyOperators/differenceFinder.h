#ifndef ONTOLOGENIUS_DIFFERENCEFINDER_H
#define ONTOLOGENIUS_DIFFERENCEFINDER_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Ontology.h"

namespace ontologenius {

class comparator_t
{
public:
  comparator_t() {concept_ = ""; }
  std::string concept_;
  std::vector<std::string> object_properties_name_;
  std::vector<std::string> object_properties_on_;

  std::vector<std::string> data_properties_name_;
  std::vector<std::string> data_properties_data_;

  std::vector<std::string> mothers_;
};

class differenceFinder
{
public:
  differenceFinder() {}

  std::vector<std::string> getDiff(Ontology* onto1, Ontology* onto2, const std::string& concept);

private:
  comparator_t toComparator(IndividualBranch_t* indiv);
  comparator_t toComparator(ClassBranch_t* class_);

  std::vector<std::string> compare(comparator_t& comp1, comparator_t& comp2);
  void compareObjects(comparator_t& comp1, comparator_t& comp2, std::vector<std::string>& res);
  void compareDatas(comparator_t& comp1, comparator_t& comp2, std::vector<std::string>& res);
  void compareMothers(comparator_t& comp1, comparator_t& comp2, std::vector<std::string>& res);

  std::vector<std::string> toValued(std::vector<ObjectPropertyBranch_t*> vect)
  {
    std::vector<std::string> res;
    for(auto it : vect)
      res.push_back(it->value());
    return res;
  }

  std::vector<std::string> toValued(std::vector<DataPropertyBranch_t*> vect)
  {
    std::vector<std::string> res;
    for(auto it : vect)
      res.push_back(it->value());
    return res;
  }

  std::vector<std::string> toValued(std::vector<IndividualBranch_t*> vect)
  {
    std::vector<std::string> res;
    for(auto it : vect)
      res.push_back(it->value());
    return res;
  }

  std::vector<std::string> toValued(std::vector<ClassBranch_t*> vect)
  {
    std::vector<std::string> res;
    for(auto it : vect)
      res.push_back(it->value());
    return res;
  }

  std::vector<std::string> toValued(std::vector<data_t> vect)
  {
    std::vector<std::string> res;
    for(auto it : vect)
      res.push_back(it.toString());
    return res;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DIFFERENCEFINDER_H
