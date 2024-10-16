#include "ontologenius/core/ontoGraphs/Branchs/IndividualBranch.h"

#include <algorithm>
#include <cstddef>

#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"

namespace ontologenius {

  int IndividualBranch::objectRelationExists(ObjectPropertyBranch* property, IndividualBranch* individual)
  {
    for(size_t i = 0; i < object_relations_.size(); i++)
    {
      if(object_relations_[i].second == individual)
        if(object_relations_[i].first == property)
          return (int)i;
    }
    return -1;
  }

  int IndividualBranch::dataRelationExists(DataPropertyBranch* property, LiteralNode* data)
  {
    for(size_t i = 0; i < data_relations_.size(); i++)
    {
      if(data_relations_[i].second == data)
        if(data_relations_[i].first == property)
          return (int)i;
    }
    return -1;
  }

  // hasUpdateOnObjectProperty
  bool IndividualBranch::hasUpdatedObjectRelation() // a renommer/ clarifier propriété qui a changé (un héritage par exemple)
  {
    return (std::find_if(object_relations_.begin(), object_relations_.end(), [](const auto& relation) { return relation.first->isUpdated(); }) != object_relations_.end());
  }

  bool IndividualBranch::hasUpdatedDataRelation()
  {
    return (std::find_if(data_relations_.begin(), data_relations_.end(), [](const auto& relation) { return relation.first->isUpdated(); }) != data_relations_.end());
  }

} // namespace ontologenius
