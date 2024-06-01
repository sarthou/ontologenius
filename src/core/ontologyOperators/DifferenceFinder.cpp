#include "ontologenius/core/ontologyOperators/DifferenceFinder.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Ontology.h"

namespace ontologenius {

  std::vector<std::string> DifferenceFinder::getDiff(Ontology* onto1, Ontology* onto2, const std::string& concept)
  {
    Comparator comp1, comp2;

    IndividualBranch* indiv_onto1 = onto1->individual_graph_.findBranchSafe(concept);
    IndividualBranch* indiv_onto2 = onto2->individual_graph_.findBranchSafe(concept);

    if((indiv_onto1 == nullptr) && (indiv_onto2 == nullptr))
    {
      ClassBranch* class_onto1 = onto1->class_graph_.findBranchSafe(concept);
      ClassBranch* class_onto2 = onto2->class_graph_.findBranchSafe(concept);
      if(class_onto2 != nullptr)
        comp2 = toComparator(class_onto2);
      if(class_onto1 != nullptr)
        comp1 = toComparator(class_onto1);
    }
    else if(indiv_onto1 == nullptr)
    {
      comp2 = toComparator(indiv_onto2);
      ClassBranch* class_onto1 = onto1->class_graph_.findBranchSafe(concept);
      if(class_onto1 != nullptr)
        comp1 = toComparator(class_onto1);
    }
    else if(indiv_onto2 == nullptr)
    {
      comp1 = toComparator(indiv_onto1);
      ClassBranch* class_onto2 = onto2->class_graph_.findBranchSafe(concept);
      if(class_onto2 != nullptr)
        comp2 = toComparator(class_onto2);
    }
    else
    {
      comp1 = toComparator(indiv_onto1);
      comp2 = toComparator(indiv_onto2);
    }

    return compare(comp1, comp2);
  }

  std::vector<std::string> DifferenceFinder::compare(Comparator& comp1, Comparator& comp2)
  {
    std::vector<std::string> res;

    compareObjects(comp1, comp2, res);
    compareDatas(comp1, comp2, res);
    compareMothers(comp1, comp2, res);

    return res;
  }

  void DifferenceFinder::compareObjects(Comparator& comp1, Comparator& comp2, std::vector<std::string>& res)
  {
    std::vector<size_t> explored_indexs;

    if(comp1.concept_.empty() == false)
    {
      for(size_t i = 0; i < comp1.object_properties_name_.size(); i++)
      {
        std::vector<size_t> found_indexs;
        for(size_t j = 0; j < comp2.object_properties_name_.size(); j++)
          if(comp1.object_properties_name_[i] == comp2.object_properties_name_[j])
          {
            found_indexs.push_back(j);
          }

        bool same = false;
        for(size_t index : found_indexs)
        {
          if(comp1.object_properties_on_[i] == comp2.object_properties_on_[index])
            same = true;
          else
            res.push_back("[-]" + comp1.concept_ + "|" + comp2.object_properties_name_[index] + "|" + comp2.object_properties_on_[index]);
        }
        if(same == false)
          res.push_back("[+]" + comp1.concept_ + "|" + comp1.object_properties_name_[i] + "|" + comp1.object_properties_on_[i]);
        explored_indexs.insert(explored_indexs.end(), found_indexs.begin(), found_indexs.end());
      }
    }

    if(comp2.concept_.empty() == false)
    {
      for(size_t i = 0; i < comp2.object_properties_name_.size(); i++)
      {
        if(std::find(explored_indexs.begin(), explored_indexs.end(), i) == explored_indexs.end())
          res.push_back("[-]" + comp2.concept_ + "|" + comp2.object_properties_name_[i] + "|" + comp2.object_properties_on_[i]);
      }
    }
  }

  void DifferenceFinder::compareDatas(Comparator& comp1, Comparator& comp2, std::vector<std::string>& res)
  {
    std::vector<size_t> explored_indexs;

    if(comp1.concept_.empty() == false)
    {
      for(size_t i = 0; i < comp1.data_properties_name_.size(); i++)
      {
        std::vector<size_t> found_indexs;
        for(size_t j = 0; j < comp2.data_properties_name_.size(); j++)
          if(comp1.data_properties_name_[i] == comp2.data_properties_name_[j])
          {
            found_indexs.push_back(j);
          }

        bool same = false;
        for(size_t index : found_indexs)
        {
          if(comp1.data_properties_data_[i] == comp2.data_properties_data_[index])
            same = true;
          else
            res.push_back("[-]" + comp1.concept_ + "|" + comp2.data_properties_name_[index] + "|" + comp2.data_properties_data_[index]);
        }
        if(same == false)
          res.push_back("[+]" + comp1.concept_ + "|" + comp1.data_properties_name_[i] + "|" + comp1.data_properties_data_[i]);
        explored_indexs.insert(explored_indexs.end(), found_indexs.begin(), found_indexs.end());
      }
    }

    if(comp2.concept_.empty() == false)
    {
      for(size_t i = 0; i < comp2.data_properties_name_.size(); i++)
      {
        if(std::find(explored_indexs.begin(), explored_indexs.end(), i) == explored_indexs.end())
          res.push_back("[-]" + comp2.concept_ + "|" + comp2.data_properties_name_[i] + "|" + comp2.data_properties_data_[i]);
      }
    }
  }

  void DifferenceFinder::compareMothers(Comparator& comp1, Comparator& comp2, std::vector<std::string>& res)
  {
    std::vector<size_t> explored_indexs;

    if(comp1.concept_.empty() == false)
    {
      for(size_t i = 0; i < comp1.mothers_.size(); i++)
      {
        std::vector<size_t> found_indexs;
        for(size_t j = 0; j < comp2.mothers_.size(); j++)
          if(comp1.mothers_[i] == comp2.mothers_[j])
          {
            found_indexs.push_back(j);
          }

        if(found_indexs.empty())
          res.push_back("[+]" + comp1.concept_ + "|isA|" + comp1.mothers_[i]);
        explored_indexs.insert(explored_indexs.end(), found_indexs.begin(), found_indexs.end());
      }
    }

    if(comp2.concept_.empty() == false)
    {
      for(size_t i = 0; i < comp2.mothers_.size(); i++)
      {
        if(std::find(explored_indexs.begin(), explored_indexs.end(), i) == explored_indexs.end())
          res.push_back("[-]" + comp2.concept_ + "|isA|" + comp2.mothers_[i]);
      }
    }
  }

  Comparator DifferenceFinder::toComparator(IndividualBranch* indiv)
  {
    Comparator comp;
    comp.concept_ = indiv->value();
    comp.object_properties_name_ = toValuedFirst(indiv->object_relations_);
    comp.object_properties_on_ = toValuedSecond(indiv->object_relations_);
    comp.data_properties_name_ = toValuedFirst(indiv->data_relations_);
    comp.data_properties_data_ = toValuedSecond(indiv->data_relations_);
    comp.mothers_ = toValued(indiv->is_a_);
    return comp;
  }

  Comparator DifferenceFinder::toComparator(ClassBranch* class_branch)
  {
    Comparator comp;
    comp.concept_ = class_branch->value();

    comp.object_properties_name_ = toValuedFirst(class_branch->object_relations_);
    comp.object_properties_on_ = toValuedSecond(class_branch->object_relations_);

    comp.data_properties_name_ = toValuedFirst(class_branch->data_relations_);
    comp.data_properties_data_ = toValuedSecond(class_branch->data_relations_);
    comp.mothers_ = toValued(class_branch->mothers_);
    return comp;
  }

} // namespace ontologenius
