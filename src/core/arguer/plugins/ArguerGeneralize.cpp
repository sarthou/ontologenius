#include "ontoloGenius/core/arguer/plugins/ArguerGeneralize.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

ArguerGeneralize::ArguerGeneralize()
{
  current_id_ = 0;
  class_per_period_ = 5;
}

void ArguerGeneralize::periodicReason()
{
  std::vector<ClassBranch_t*> classes = ontology_->class_graph_.get();
  if(classes.size() > 0)
  {
    for(size_t i = 0; i < class_per_period_; i++)
    {
      PropertiesCounter<DataPropertyBranch_t*, std::string> data_counter;
      PropertiesCounter<ObjectPropertyBranch_t*, ClassBranch_t*> object_counter;

      std::unordered_set<ClassBranch_t*> down_set = ontology_->class_graph_.getDownPtr(classes[current_id_], 1);
      down_set.erase(classes[current_id_]);
      for(auto down : down_set)
      {
        for(size_t j = 0; j < down->data_properties_name_.size(); j++)
          data_counter.add(down, down->data_properties_name_[j], down->data_properties_data_[j].toString());

        for(size_t j = 0; j < down->object_properties_name_.size(); j++)
          object_counter.add(down, down->object_properties_name_[j], down->object_properties_on_[j]);
      }

      std::vector<DataPropertyBranch_t*> data_properties;
      std::vector<std::string> data_datas;
      std::vector<std::vector<ClassBranch_t*> > data_classes;
      data_counter.get(data_properties, data_datas, data_classes, down_set.size());

      std::vector<ObjectPropertyBranch_t*> object_properties;
      std::vector<ClassBranch_t*> object_datas;
      std::vector<std::vector<ClassBranch_t*> > object_classes;
      object_counter.get(object_properties, object_datas, object_classes, down_set.size());

      for(size_t tmp = 0; tmp < data_properties.size(); tmp++)
        std::cout << classes[current_id_]->value() << " infer " << data_properties[tmp]->value() << " : " << data_datas[tmp] << std::endl;

      for(size_t tmp = 0; tmp < object_properties.size(); tmp++)
        std::cout << classes[current_id_]->value() << " infer " << object_properties[tmp]->value() << " : " << object_datas[tmp] << std::endl;

      current_id_++;
      if(current_id_ >= classes.size())
        current_id_ = 0;
    }
  }
}

std::string ArguerGeneralize::getName()
{
  return "arguer generalize";
}

std::string ArguerGeneralize::getDesciption()
{
  return "This arguer aims to infer new knowledge by generalizing explicit relationships between concepts.";
}

PLUGINLIB_EXPORT_CLASS(ArguerGeneralize, ArguerInterface)
