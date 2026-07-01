#include "ontologenius/core/feeder/Versionor.h"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ontologenius/core/feeder/FeedStorage.h"
#include "ontologenius/core/feeder/VersionNode.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  Versionor::Versionor(FeedStorage* storage) : activated_(false),
                                               storage_(storage),
                                               order_(0)
  {
    auto* first_node = new VersionNode(order_, "0");
    order_++;
    current_node_ = new VersionNode(order_, first_node);
    order_++;
    nodes_[first_node->getId()] = first_node;
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
    auto node_it = getNode(id);
    if(node_it == nodes_.end())
      return false;

    auto datas = getDataBetween(current_node_, node_it->second);

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

  std::string Versionor::getCurrentCommit()
  {
    if(current_node_ == nullptr)
      return "";
    else
      return (current_node_->getPrev() == nullptr) ? "" : current_node_->getPrev()->getId();
  }

  size_t Versionor::getNbData()
  {
    return (current_node_ == nullptr) ? 0 : current_node_->getNbData();
  }

  bool Versionor::areSameStates(const std::string& from_node, const std::string& to_node)
  {
    auto from_node_it = getNode(from_node);
    if(from_node_it == nodes_.end())
      return false;

    VersionNode* to_node_ptr = nullptr;
    if(to_node.empty())
      to_node_ptr = current_node_;
    else
    {
      auto to_node_it = nodes_.find(to_node);
      if(to_node_it == nodes_.end())
        return false;
      else
        to_node_ptr = to_node_it->second;
    }

    std::vector<Feed_t> datas = getDataBetween(from_node_it->second, to_node_ptr);
    std::vector<bool> used(datas.size(), false);

    for(size_t i = 0; i < datas.size(); i++)
    {
      if(used[i])
        continue;

      int count = 1;
      for(size_t j = i + 1; j < datas.size(); j++)
      {
        if((used[j] == false) && datas[i].sameData(datas[j]))
        {
          used[j] = true;
          count += datas[i].isInvertAction(datas[j]) ? -1 : 1;
        }
      }

      if(count != 0)
        return false;
    }

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

  std::vector<Feed_t> Versionor::getDataBetween(VersionNode* from_node, VersionNode* to_node)
  {
    std::vector<VersionNode*> goal_prevs = getPrevs(to_node);
    std::vector<VersionNode*> current_prevs;
    VersionNode* pivot = from_node;
    while(std::find(goal_prevs.begin(), goal_prevs.end(), pivot) == goal_prevs.end())
    {
      current_prevs.push_back(pivot);
      pivot = pivot->getPrev();
      if(pivot == nullptr)
        return {};
    }

    std::vector<Feed_t> datas;
    for(auto* current_prev : current_prevs)
      current_prev->appendDatasInvert(datas);

    bool found_pivot = false;
    for(auto* node : goal_prevs)
    {
      if(node == pivot)
        found_pivot = true;
      else if(found_pivot)
        node->appendDatasDirect(datas);
    }

    return datas;
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

  std::unordered_map<std::string, VersionNode*>::iterator Versionor::getNode(const std::string& id)
  {
    if(!activated_)
      return nodes_.end();

    if(id.empty())
      return nodes_.end();

    return nodes_.find(id);
  }

} // namespace ontologenius
