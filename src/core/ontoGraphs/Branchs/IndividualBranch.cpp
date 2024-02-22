#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"

#include <algorithm>

namespace ontologenius {

int IndividualBranch_t::objectRelationExists(ObjectPropertyBranch_t* property, IndividualBranch_t* individual)
{
  for(size_t i = 0; i < object_relations_.size(); i++)
  {
    if(object_relations_[i].second == individual)
      if(object_relations_[i].first == property)
        return i;
  }
  return -1;
}

int IndividualBranch_t::dataRelationExists(DataPropertyBranch_t* property, LiteralNode* data)
{
  for(size_t i = 0; i < data_relations_.size(); i++)
  {
    if(data_relations_[i].second == data)
      if(data_relations_[i].first == property)
        return i;
  }
  return -1;
}

bool IndividualBranch_t::hasUpdatedObjectRelation()
{
  return (std::find_if(object_relations_.begin(), object_relations_.end(), [](const auto& relation) { return relation.first->updated_; }) != object_relations_.end());
}

bool IndividualBranch_t::hasUpdatedDataRelation()
{
  return (std::find_if(data_relations_.begin(), data_relations_.end(), [](const auto& relation) { return relation.first->updated_; }) != data_relations_.end());
}

} // namespace ontologenius
