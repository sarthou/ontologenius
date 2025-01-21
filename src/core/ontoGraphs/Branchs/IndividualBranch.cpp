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

  bool IndividualBranch::hasUpdatedObjectRelation() // one of the object properties used by the individual has got its inheritance updated
  {
    return (std::find_if(object_relations_.begin(), object_relations_.end(), [](const auto& relation) { return (relation.first->isUpdated() && relation.first->mothers_.isUpdated()); }) != object_relations_.end());
  }

  bool IndividualBranch::hasUpdatedDataRelation() // one of the data properties used by the individual has got its inheritance updated
  {
    return (std::find_if(data_relations_.begin(), data_relations_.end(), [](const auto& relation) { return (relation.first->isUpdated() && relation.first->mothers_.isUpdated()); }) != data_relations_.end());
  }

  bool IndividualBranch::hasUpdatedInheritanceRelation() // one of the classes of the individual's inheritance has been updated
  {
    return (std::find_if(is_a_.begin(), is_a_.end(), [](const auto& class_member) { return (class_member.elem->isUpdated() && class_member.elem->mothers_.isUpdated()); }) != is_a_.end());
  }

} // namespace ontologenius
