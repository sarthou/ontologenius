#include "ontologenius/core/reasoner/plugins/ReasonerGeneralize.h"

#include <pluginlib/class_list_macros.h>

namespace ontologenius {

ReasonerGeneralize::ReasonerGeneralize()
{
  current_id_ = 0;
  class_per_period_ = 25;

  min_count_ = 2;
  min_percent_ = 0.6;
}

void ReasonerGeneralize::setParameter(const std::string& name, const std::string& value)
{
  std::string::size_type sz;
  if(name == "min_count")
    min_count_ = std::stoi(value,&sz);
  else if(name == "min_percent")
    min_percent_ = std::stof(value,&sz);
}

void ReasonerGeneralize::periodicReason()
{
  std::vector<ClassBranch_t*> classes = ontology_->class_graph_.getSafe();

  if(classes.size() > 0)
  {
    if(current_id_ >= classes.size())
      current_id_ = 0;

    size_t first_id = current_id_;
    for(size_t i = 0; i < class_per_period_; i++)
    {
      std::unordered_set<ClassBranch_t*> down_set = ontology_->class_graph_.getDownPtrSafe(classes[current_id_], 1);
      down_set.erase(classes[current_id_]);

      std::unordered_set<IndividualBranch_t*> indiv_down_set = ontology_->class_graph_.getDownIndividualPtrSafe(classes[current_id_]);
      std::shared_lock<std::shared_timed_mutex> lock_indiv_shared(ontology_->individual_graph_.mutex_);
      std::shared_lock<std::shared_timed_mutex> lock_shared(ontology_->class_graph_.mutex_);

      PropertiesCounter<DataPropertyBranch_t*, std::string> data_counter(min_count_, min_percent_);
      PropertiesCounter<ObjectPropertyBranch_t*, ClassBranch_t*> object_counter;

      for(auto down : down_set)
      {
        for(auto& data_relation : down->data_relations_)
          data_counter.add(data_relation.first, data_relation.second.toString());

        for(auto& object_relation : down->object_relations_)
          object_counter.add(object_relation.first, object_relation.second);
      }

      for(auto down : indiv_down_set)
      {
        for(auto& data_relation : down->data_relations_)
          data_counter.add(data_relation.first, data_relation.second.toString());
      }

      lock_shared.unlock();
      lock_indiv_shared.unlock();
      std::lock_guard<std::shared_timed_mutex> lock_indiv(ontology_->individual_graph_.mutex_);
      std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);

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
      if(current_id_ == first_id)
        break;
    }
  }
}

void ReasonerGeneralize::setDeduced(ClassBranch_t* me, std::vector<DataPropertyBranch_t*> properties, std::vector<std::string> datas)
{
  std::unordered_set<size_t> deduced_indexs;
  for(size_t i = 0; i < me->data_relations_.size(); i++)
    if(me->data_relations_[i] < 0.51) // deduced = 0.5
      deduced_indexs.insert(i);

  for(size_t prop = 0; prop < properties.size(); prop++)
  {
    int index = -1;

    for(size_t prop_i = 0; prop_i < me->data_relations_.size(); prop_i++)
      if(me->data_relations_[prop_i].first == properties[prop])
      {
        data_t tmp;
        tmp.set(datas[prop]);

        if(me->data_relations_[prop_i].second.toString() != tmp.toString())
          notifications_.push_back("[CHANGE]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]);

        index = prop_i;
        deduced_indexs.erase(index);
        me->data_relations_[prop_i].second = tmp;
        me->data_relations_[prop_i].probability = 0.5;
      }

    if(index == -1)
    {
      notifications_.push_back("[NEW]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]);
      data_t tmp;
      tmp.set(datas[prop]);
      me->data_relations_.emplace_back(properties[prop], tmp, 0.5);
      properties[prop]->annotation_usage_ = true;
    }
  }

  if(deduced_indexs.size() != 0)
  {
    size_t deleted = 0;
    for(auto i : deduced_indexs)
    {
      notifications_.push_back("[DELETE]" + me->value() + ">" + me->data_relations_[i- deleted].first->value() + ":" + me->data_relations_[i- deleted].second.toString());
      me->data_relations_.erase(me->data_relations_.begin() + i - deleted);
      deleted++;
    }
  }
}

void ReasonerGeneralize::setDeduced(ClassBranch_t* me, std::vector<ObjectPropertyBranch_t*> properties, std::vector<ClassBranch_t*> datas)
{
  std::unordered_set<size_t> deduced_indexs;
  for(size_t i = 0; i < me->object_relations_.size(); i++)
    if(me->object_relations_[i] < 0.51) // deduced = 0.5
      deduced_indexs.insert(i);

  for(size_t prop = 0; prop < properties.size(); prop++)
  {
    int index = -1;

    for(size_t prop_i = 0; prop_i < me->object_relations_.size(); prop_i++)
      if(me->object_relations_[prop_i].first == properties[prop])
      {
        if(me->object_relations_[prop_i].second != datas[prop])
          notifications_.push_back("[CHANGE]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]->value());

        index = prop_i;
        deduced_indexs.erase(index);
        me->object_relations_[prop_i].second = datas[prop];
        me->object_relations_[prop_i].probability = 0.5;
      }

    if(index == -1)
    {
      notifications_.push_back("[NEW]" + me->value() + ">" + properties[prop]->value() + ":" + datas[prop]->value());
      me->object_relations_.emplace_back(properties[prop], datas[prop], 0.5);
    }
  }

  if(deduced_indexs.size() != 0)
  {
    size_t deleted = 0;
    for(auto i : deduced_indexs)
    {
      notifications_.push_back("[DELETE]" + me->value() + ">" + me->object_relations_[i- deleted].first->value() + ":" + me->object_relations_[i- deleted].second->value());
      me->object_relations_.erase(me->object_relations_.begin() + i - deleted);
      deleted++;
    }
  }
}

std::string ReasonerGeneralize::getName()
{
  return "reasoner generalize";
}

std::string ReasonerGeneralize::getDesciption()
{
  return "This reasoner aims to infer new knowledge by generalizing explicit relationships between concepts.";
}

PLUGINLIB_EXPORT_CLASS(ReasonerGeneralize, ReasonerInterface)

} // namespace ontologenius
