#include "ontologenius/core/feeder/Versionor.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  Versionor::Versionor(FeedStorage* storage)
  {
    order_ = 0;
    storage_ = storage;
    auto* first_node = new VersionNode(order_, "0");
    order_++;
    current_node_ = new VersionNode(order_, first_node);
    order_++;
    nodes_[first_node->getId()] = first_node;
    activated_ = false;
  }

  Versionor::~Versionor()
  {
    if(current_node_ != nullptr)
      if(current_node_->defined() == false)
        delete current_node_;

    for(auto& it : nodes_)
      delete it.second;
  }

  void Versionor::insert(Feed_t data)
  {
    if(activated_)
    {
      data.checkout_ = true;
      current_node_->insert(data);
    }
  }

  bool Versionor::commit(const std::string& id)
  {
    if(!activated_)
      return false;

    if(id.empty())
      return false;
    if(nodes_.find(id) != nodes_.end())
      return false;

    VersionNode* old_node = current_node_;
    old_node->setId(id);
    current_node_ = new VersionNode(order_, old_node);
    order_++;
    nodes_[old_node->getId()] = old_node;
    return true;
  }

  bool Versionor::checkout(const std::string& id)
  {
    if(!activated_)
      return false;

    if(id.empty())
      return false;

    auto node_it = nodes_.find(id);
    if(node_it == nodes_.end())
      return false;

    std::vector<VersionNode*> goal_prevs = getPrevs(node_it->second);
    std::vector<VersionNode*> current_prevs;
    VersionNode* pivot = current_node_;
    while(std::find(goal_prevs.begin(), goal_prevs.end(), pivot) == goal_prevs.end())
    {
      current_prevs.push_back(pivot);
      pivot = pivot->getPrev();
      if(pivot == nullptr)
        break;
    }

    if(pivot == nullptr)
      return false;

    current_prevs.push_back(pivot);

    std::vector<Feed_t> datas;
    for(auto& current_prev : current_prevs)
    {
      if(current_prev == pivot)
        break;
      current_prev->appendDatasInvert(datas);
    }

    bool found_pivot = false;
    for(auto& goal_prev : goal_prevs)
    {
      if(goal_prev == pivot)
      {
        found_pivot = true;
        continue;
      }
      else if(found_pivot == false)
        continue;
      else
        goal_prev->appendDatasDirect(datas);
    }

    if(current_node_->defined() == false)
    {
      current_node_->unlinkFromPrev();
      delete current_node_;
      current_node_ = nullptr;
    }

    current_node_ = new VersionNode(order_, node_it->second);
    order_++;
    if(storage_ != nullptr)
      storage_->add(datas);

    return true;
  }

  void Versionor::exportToXml(const std::string& path)
  {
    if(!activated_)
      return;

    const std::string xml = nodes_["0"]->toXml();

    if(path.empty())
    {
      std::cout << xml;
      return;
    }

    FILE* file = fopen(path.c_str(), "w");
    if(file == nullptr)
    {
      Display::error("Fail to open file : ", false);
      std::cout << path << std::endl;
      return;
    }

    fwrite(xml.c_str(), sizeof(char), xml.size(), file);
    fclose(file);
  }

  std::vector<VersionNode*> Versionor::getPrevs(VersionNode* from_node)
  {
    std::vector<VersionNode*> goal_prevs;
    VersionNode* tmp_prev = from_node;
    while(tmp_prev != nullptr)
    {
      goal_prevs.push_back(tmp_prev);
      tmp_prev = tmp_prev->getPrev();
    }
    std::reverse(goal_prevs.begin(), goal_prevs.end());
    return goal_prevs;
  }

} // namespace ontologenius
