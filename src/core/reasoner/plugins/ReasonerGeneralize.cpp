#include "ontologenius/core/reasoner/plugins/ReasonerGeneralize.h"

#include <cstddef>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"

namespace ontologenius {

  ReasonerGeneralize::ReasonerGeneralize() : current_id_(0),
                                             class_per_period_(25),
                                             min_count_(0),
                                             min_percent_(0.6)
  {}

  void ReasonerGeneralize::setParameter(const std::string& name, const std::string& value)
  {
    std::string::size_type sz = 0;
    if(name == "min_count")
      min_count_ = std::stoi(value, &sz);
    else if(name == "min_percent")
      min_percent_ = std::stof(value, &sz);
  }

  bool ReasonerGeneralize::periodicReason()
  {
    std::vector<ClassBranch*> classes = ontology_->class_graph_.getSafe();

    if(classes.empty() == false)
    {
      if(current_id_ >= classes.size())
        current_id_ = 0;

      size_t first_id = current_id_;
      for(size_t i = 0; i < class_per_period_; i++)
      {
        std::unordered_set<ClassBranch*> down_set = ontology_->class_graph_.getDownPtrSafe(classes[current_id_], 1);
        down_set.erase(classes[current_id_]);

        std::unordered_set<IndividualBranch*> indiv_down_set = ontology_->class_graph_.getDownIndividualPtrSafe(classes[current_id_], 0);
        std::shared_lock<std::shared_timed_mutex> lock_indiv_shared(ontology_->individual_graph_.mutex_);
        std::shared_lock<std::shared_timed_mutex> lock_shared(ontology_->class_graph_.mutex_);

        PropertiesCounter<DataPropertyBranch*, LiteralNode*> data_counter(min_count_, min_percent_);
        PropertiesCounter<ObjectPropertyBranch*, ClassBranch*> object_counter;

        for(auto* down : down_set)
        {
          for(auto& data_relation : down->data_relations_)
            data_counter.add(data_relation.first, data_relation.second);

          for(auto& object_relation : down->object_relations_)
            object_counter.add(object_relation.first, object_relation.second);
        }

        for(auto* down : indiv_down_set)
        {
          for(auto& data_relation : down->data_relations_)
            data_counter.add(data_relation.first, data_relation.second);
        }

        lock_shared.unlock();
        lock_indiv_shared.unlock();
        std::lock_guard<std::shared_timed_mutex> lock_indiv(ontology_->individual_graph_.mutex_);
        std::lock_guard<std::shared_timed_mutex> lock(ontology_->class_graph_.mutex_);

        auto data_properties = data_counter.get();
        setDeduced(classes[current_id_], data_properties);

        auto object_properties = object_counter.get();
        setDeduced(classes[current_id_], object_properties);

        current_id_++;
        if(current_id_ >= classes.size())
          current_id_ = 0;
        if(current_id_ == first_id)
          break;
      }
    }

    return (notifications_.empty() == false);
  }

  void ReasonerGeneralize::setDeduced(ClassBranch* me, std::vector<std::tuple<DataPropertyBranch*, LiteralNode*, float>> properties)
  {
    std::unordered_set<size_t> deduced_indexs;
    for(size_t i = 0; i < me->data_relations_.size(); i++)
      if(me->data_relations_[i] < 1.0)
        deduced_indexs.insert(i);

    for(auto& property : properties)
    {
      int index = -1;

      for(size_t prop_i = 0; prop_i < me->data_relations_.size(); prop_i++)
        if(me->data_relations_[prop_i].first == std::get<0>(property))
        {
          index = (int)prop_i;
          deduced_indexs.erase(index);

          if(me->data_relations_[prop_i].probability < 1.0)
          {
            if(me->data_relations_[prop_i].second != std::get<1>(property))
              notifications_.emplace_back(notification_info, "[CHANGE]" + me->value() + ">" + std::get<0>(property)->value() + ":" + std::get<1>(property)->value());

            me->data_relations_[prop_i].second = std::get<1>(property);
            me->data_relations_[prop_i].probability = (float)std::get<2>(property) - 0.01;
          }
        }

      if(index == -1)
      {
        notifications_.emplace_back(notification_info, "[NEW]" + me->value() + ">" + std::get<0>(property)->value() + ":" + std::get<1>(property)->value());
        me->data_relations_.emplace_back(std::get<0>(property), std::get<1>(property), (float)std::get<2>(property) - 0.01);
        std::get<0>(property)->annotation_usage_ = true;
      }
    }

    if(deduced_indexs.empty() == false)
    {
      size_t deleted = 0;
      for(auto i : deduced_indexs)
      {
        notifications_.emplace_back(notification_info, "[DELETE]" + me->value() + ">" + me->data_relations_[i - deleted].first->value() + ":" + me->data_relations_[i - deleted].second->value());
        me->data_relations_.erase(me->data_relations_.begin() + i - deleted);
        deleted++;
      }
    }
  }

  void ReasonerGeneralize::setDeduced(ClassBranch* me, std::vector<std::tuple<ObjectPropertyBranch*, ClassBranch*, float>> properties)
  {
    std::unordered_set<size_t> deduced_indexs;
    for(size_t i = 0; i < me->object_relations_.size(); i++)
      if(me->object_relations_[i] < 1.0)
        deduced_indexs.insert(i);

    for(auto& property : properties)
    {
      int index = -1;

      for(size_t prop_i = 0; prop_i < me->object_relations_.size(); prop_i++)
        if(me->object_relations_[prop_i].first == std::get<0>(property))
        {
          index = prop_i;
          deduced_indexs.erase(index);
          if(me->object_relations_[prop_i].probability < 1.0)
          {
            if(me->object_relations_[prop_i].second != std::get<1>(property))
              notifications_.emplace_back(notification_info, "[CHANGE]" + me->value() + ">" + std::get<0>(property)->value() + ":" + std::get<1>(property)->value());

            me->object_relations_[prop_i].second = std::get<1>(property);
            me->object_relations_[prop_i].probability = (float)std::get<2>(property) - 0.01;
          }
        }

      if(index == -1)
      {
        notifications_.emplace_back(notification_info, "[NEW]" + me->value() + ">" + std::get<0>(property)->value() + ":" + std::get<1>(property)->value());
        me->object_relations_.emplace_back(std::get<0>(property), std::get<1>(property), (float)std::get<2>(property) - 0.01);
      }
    }

    if(deduced_indexs.empty() == false)
    {
      size_t deleted = 0;
      for(auto i : deduced_indexs)
      {
        notifications_.emplace_back(notification_info, "[DELETE]" + me->value() + ">" + me->object_relations_[i - deleted].first->value() + ":" + me->object_relations_[i - deleted].second->value());
        me->object_relations_.erase(me->object_relations_.begin() + i - deleted);
        deleted++;
      }
    }
  }

  std::string ReasonerGeneralize::getName()
  {
    return "reasoner generalize";
  }

  std::string ReasonerGeneralize::getDescription()
  {
    return "This reasoner aims to infer new knowledge by generalizing explicit relationships between concepts.";
  }

} // namespace ontologenius

PLUGINLIB_EXPORT_CLASS(ontologenius::ReasonerGeneralize, ontologenius::ReasonerInterface)
