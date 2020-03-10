#include "ontologenius/core/feeder/Versionor.h"

#include <algorithm>
#include <iostream>

#include "ontologenius/graphical/Display.h"

namespace ontologenius {

Versionor::Versionor(FeedStorage* storage)
{
  storage_ = storage;
  Version_node* first_node = new Version_node("0");
  current_node_ = new Version_node(first_node);
  nodes_[first_node->getId()] = first_node;
  activated_ = false;
}

Versionor::~Versionor()
{
  if(current_node_ != nullptr)
    if(current_node_->defined() == false)
      delete current_node_;

  for(auto& it : nodes_)
    if(it.second != nullptr)
      delete it.second;
}

void Versionor::insert(feed_t data)
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

  if(id == "")
    return false;
  if(nodes_.find(id) != nodes_.end())
    return false;

  Version_node* old_node = current_node_;
  old_node->setId(id);
  current_node_ = new Version_node(old_node);
  nodes_[old_node->getId()] = old_node;
  return true;
}

bool Versionor::checkout(const std::string& id)
{
  if(!activated_)
    return false;

  if(id == "")
    return false;

  auto node_it = nodes_.find(id);
  if(node_it == nodes_.end())
    return false;

  std::vector<Version_node*> goal_prevs = getPrevs(node_it->second);
  std::vector<Version_node*> current_prevs;
  Version_node* pivot = current_node_;
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

  std::vector<feed_t> datas;
  for(auto& current_prev : current_prevs)
  {
    if(current_prev == pivot)
      break;
    std::vector<feed_t> tmp_data = current_prev->getDatasInvert();
    datas.insert(datas.end(), tmp_data.begin(), tmp_data.end());
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
    {
      std::vector<feed_t> tmp_data = goal_prev->getDatasDirect();
      datas.insert(datas.end(), tmp_data.begin(), tmp_data.end());
    }
  }

  if(current_node_->defined() == false)
  {
    current_node_->unlinkFromPrev();
    delete current_node_;
    current_node_ = nullptr;
  }

  current_node_ = new Version_node(node_it->second);
  if(storage_ != nullptr)
    storage_->add(datas);

  return true;
}

void Versionor::exportToXml(const std::string& path)
{
  if(!activated_)
    return;

  std::string xml = nodes_["0"]->toXml();

  if(path == "")
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

std::vector<Version_node*> Versionor::getPrevs(Version_node* from_node)
{
  std::vector<Version_node*> goal_prevs;
  Version_node* tmp_prev = from_node;
  while(tmp_prev != nullptr)
  {
    goal_prevs.push_back(tmp_prev);
    tmp_prev = tmp_prev->getPrev();
  }
  std::reverse(goal_prevs.begin(), goal_prevs.end());
  return goal_prevs;
}

} // namespace ontologenius
