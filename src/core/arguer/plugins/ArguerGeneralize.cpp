#include "ontoloGenius/core/arguer/plugins/ArguerGeneralize.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

ArguerGeneralize::ArguerGeneralize()
{
  current_id_ = 0;
  class_per_period_ = 25;
}

void ArguerGeneralize::periodicReason()
{
  std::vector<ClassBranch_t*> classes = ontology_->class_graph_.get();
  if(classes.size() > 0)
  {
    for(size_t i = 0; i < class_per_period_; i++)
    {
      std::unordered_set<ClassBranch_t*> down_set = ontology_->class_graph_.getDownPtr(classes[current_id_], 1);
      down_set.erase(classes[current_id_]);

      std::unordered_set<IndividualBranch_t*> indiv_down_set = ontology_->class_graph_.getDownIndividualPtr(classes[current_id_]);

      std::lock_guard<std::shared_timed_mutex> lock_indiv(ontology_->individual_graph_.mutex_);
      std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);
      
      PropertiesCounter<DataPropertyBranch_t*, std::string> data_counter;
      PropertiesCounter<ObjectPropertyBranch_t*, ClassBranch_t*> object_counter;

      for(auto down : down_set)
      {
        for(size_t j = 0; j < down->data_properties_name_.size(); j++)
          data_counter.add(down->data_properties_name_[j], down->data_properties_data_[j].toString());

        for(size_t j = 0; j < down->object_properties_name_.size(); j++)
          object_counter.add(down->object_properties_name_[j], down->object_properties_on_[j]);
      }

      for(auto down : indiv_down_set)
      {
        for(size_t j = 0; j < down->data_properties_name_.size(); j++)
          data_counter.add(down->data_properties_name_[j], down->data_properties_data_[j].toString());
      }

      std::vector<DataPropertyBranch_t*> data_properties;
      std::vector<std::string> data_datas;
      data_counter.get(data_properties, data_datas);
      setDeduced(classes[current_id_], data_properties, data_datas);

      std::vector<ObjectPropertyBranch_t*> object_properties;
      std::vector<ClassBranch_t*> object_datas;
      object_counter.get(object_properties, object_datas);
      setDeduced(classes[current_id_], object_properties, object_datas);

      current_id_++;
      if(current_id_ >= classes.size())
        current_id_ = 0;
    }
  }
}

void ArguerGeneralize::setDeduced(ClassBranch_t* me, std::vector<DataPropertyBranch_t*> properties, std::vector<std::string> datas)
{
  std::unordered_set<size_t> deduced_indexs;
  for(size_t i = 0; i < me->data_properties_deduced_.size(); i++)
    if(me->data_properties_deduced_[i] == true)
      deduced_indexs.insert(i);

  for(size_t prop = 0; prop < properties.size(); prop++)
  {
    int index = -1;
    for(size_t prop_i = 0; prop_i < me->steady_.data_properties_name_.size(); prop_i++)
      if(me->steady_.data_properties_name_[prop_i] == properties[prop])
      {
        //the property is already know
        index = prop_i;
        break;
      }

    if(index == -1)
    {
      for(size_t prop_i = 0; prop_i < me->data_properties_name_.size(); prop_i++)
        if(me->data_properties_name_[prop_i] == properties[prop])
        {
          data_t tmp;
          tmp.set(datas[prop]);

          if(me->data_properties_data_[prop_i].toString() != tmp.toString())
            notifications_.push_back("[CHANGE]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]);

          index = prop_i;
          deduced_indexs.erase(index);
          me->data_properties_data_[prop_i] = tmp;
          me->data_properties_deduced_[prop_i] = true;
        }

      if(index == -1)
      {
        notifications_.push_back("[NEW]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]);
        me->data_properties_name_.push_back(properties[prop]);
        data_t tmp;
        tmp.set(datas[prop]);
        me->data_properties_data_.push_back(tmp);
        me->data_properties_deduced_.push_back(true);
      }
    }
  }

  if(deduced_indexs.size() != 0)
  {
    size_t deleted = 0;
    for(auto i : deduced_indexs)
    {
      notifications_.push_back("[DELETE]" + me->value() + ">" + me->data_properties_name_[i- deleted]->value() + ":" + me->data_properties_data_[i- deleted].toString());
      me->data_properties_name_.erase(me->data_properties_name_.begin() + i - deleted);
      me->data_properties_data_.erase(me->data_properties_data_.begin() + i - deleted);
      me->data_properties_deduced_.erase(me->data_properties_deduced_.begin() + i - deleted);
      deleted++;
    }
  }
}

void ArguerGeneralize::setDeduced(ClassBranch_t* me, std::vector<ObjectPropertyBranch_t*> properties, std::vector<ClassBranch_t*> datas)
{
  std::unordered_set<size_t> deduced_indexs;
  for(size_t i = 0; i < me->object_properties_deduced_.size(); i++)
    if(me->object_properties_deduced_[i] == true)
      deduced_indexs.insert(i);

  for(size_t prop = 0; prop < properties.size(); prop++)
  {
    int index = -1;
    for(size_t prop_i = 0; prop_i < me->steady_.object_properties_name_.size(); prop_i++)
      if(me->steady_.object_properties_name_[prop_i] == properties[prop])
      {
        //the property is already know
        index = prop_i;
        break;
      }

      if(index == -1)
      {
        for(size_t prop_i = 0; prop_i < me->object_properties_name_.size(); prop_i++)
          if(me->object_properties_name_[prop_i] == properties[prop])
          {
            if(me->object_properties_on_[prop_i] != datas[prop])
            notifications_.push_back("[CHANGE]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]->value());
            index = prop_i;
            deduced_indexs.erase(index);
            me->object_properties_on_[prop_i] = datas[prop];
            me->object_properties_deduced_[prop_i] = true;
          }

        if(index == -1)
        {
          notifications_.push_back("[NEW]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]->value());
          me->object_properties_name_.push_back(properties[prop]);
          me->object_properties_on_.push_back(datas[prop]);
          me->object_properties_deduced_.push_back(true);
        }
      }
  }

  if(deduced_indexs.size() != 0)
  {
    size_t deleted = 0;
    for(auto i : deduced_indexs)
    {
      notifications_.push_back("[DELETE]" + me->value() + ">" + me->object_properties_name_[i- deleted]->value() + ":" + me->object_properties_on_[i- deleted]->value());
      me->object_properties_name_.erase(me->object_properties_name_.begin() + i - deleted);
      me->object_properties_on_.erase(me->object_properties_on_.begin() + i - deleted);
      me->object_properties_deduced_.erase(me->object_properties_deduced_.begin() + i - deleted);
      deleted++;
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
