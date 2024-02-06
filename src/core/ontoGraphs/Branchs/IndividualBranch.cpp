#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"

#include <algorithm>

namespace ontologenius {

int IndividualBranch_t::objectRelationExists(ObjectPropertyBranch_t* property, IndividualBranch_t* individual)
{
  for(size_t i = 0; i < object_relations_.size(); i++)
  {
    if(object_relations_[i].first == property)
      if(object_relations_[i].second == individual)
        return i;
  }
  return -1;
}

int IndividualBranch_t::dataRelationExists(DataPropertyBranch_t* property, LiteralNode* data)
{
  for(size_t i = 0; i < data_relations_.size(); i++)
  {
    if(data_relations_[i].first == property)
      if(data_relations_[i].second == data)
        return i;
  }
  return -1;
}

} // namespace ontologenius
