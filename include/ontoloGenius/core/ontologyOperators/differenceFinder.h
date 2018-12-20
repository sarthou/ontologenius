#ifndef DIFFERENCEFINDER_H
#define DIFFERENCEFINDER_H

#include <string>
#include <vector>

class Ontology;
class IndividualBranch_t;
class ClassBranch_t;
class ObjectPropertyBranch_t;
class DataPropertyBranch_t;

class ValuedNode;
struct data_t;

class comparator_t
{
public:
  comparator_t() {concept_ = nullptr; }
  ValuedNode* concept_;
  std::vector<ValuedNode*> object_properties_name_;
  std::vector<ValuedNode*> object_properties_on_;

  std::vector<ValuedNode*> data_properties_name_;
  std::vector<data_t> data_properties_data_;

  std::vector<ValuedNode*> mothers_;
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

  std::vector<ValuedNode*> toValued(std::vector<ObjectPropertyBranch_t*> vect)
  {
    std::vector<ValuedNode*> res;
    for(auto it : vect)
      res.push_back((ValuedNode*)it);
    return res;
  }

  std::vector<ValuedNode*> toValued(std::vector<DataPropertyBranch_t*> vect)
  {
    std::vector<ValuedNode*> res;
    for(auto it : vect)
      res.push_back((ValuedNode*)it);
    return res;
  }

  std::vector<ValuedNode*> toValued(std::vector<IndividualBranch_t*> vect)
  {
    std::vector<ValuedNode*> res;
    for(auto it : vect)
      res.push_back((ValuedNode*)it);
    return res;
  }

  std::vector<ValuedNode*> toValued(std::vector<ClassBranch_t*> vect)
  {
    std::vector<ValuedNode*> res;
    for(auto it : vect)
      res.push_back((ValuedNode*)it);
    return res;
  }
};

#endif // DIFFERENCEFINDER_H
