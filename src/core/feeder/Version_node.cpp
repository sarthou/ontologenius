#include "ontologenius/core/feeder/Version_node.h"

#include <numeric>

namespace ontologenius {

size_t Version_node::global_order_id_ = 0;

Version_node::Version_node(size_t order, Version_node* prev)
{
  prev_ = prev;
  if(prev_ != nullptr)
    prev_->addNext(this);
  id_ = "";
  order_id_ = order;
}

Version_node::Version_node(size_t order, const std::string& id) : id_(id),
                                                                  order_id_(order),
                                                                  prev_(nullptr)
{}

std::vector<feed_t> Version_node::getDatasDirect()
{
  return datas_;
}

std::vector<feed_t> Version_node::getDatasInvert()
{
  std::vector<feed_t> datas = datas_;
  std::reverse(datas.begin(), datas.end());
  for(auto& data : datas)
    data.action_ = (data.action_ == action_add) ? action_del : action_add;
  return datas;
}

void Version_node::appendDatasDirect(std::vector<feed_t>& datas)
{
  datas.insert(datas.end(), datas_.begin(), datas_.end());
}

void Version_node::appendDatasInvert(std::vector<feed_t>& datas)
{
  std::vector<feed_t> tmp_datas = datas_;
  for(auto& data : tmp_datas)
    data.action_ = (data.action_ == action_add) ? action_del : action_add;
  datas.insert(datas.end(), tmp_datas.rbegin(), tmp_datas.rend());
}

void Version_node::unlinkFromPrev()
{
  if(prev_ != nullptr)
  {
    auto it = std::find(prev_->nexts_.begin(), prev_->nexts_.end(), this);
    prev_->nexts_.erase(it);
  }
}

void Version_node::print(int level)
{
  for(auto& data : datas_)
    std::cout << getSpaces(level) << std::string((data.action_ == action_add) ? "+" : "-") << data.from_ << ":" << data.prop_ << ":" << data.on_ << std::endl;
  std::cout << getSpaces(level) << "--" << id_ << "--" << std::endl;
  level++;
  for(auto& next : nexts_)
    next->print(level);
}

std::string Version_node::toXml(int level)
{
  std::string xml;
  xml += getSpaces(level) + "<Node id=\"" + id_ + "\">\n";
  xml += getSpaces(level + 1) + orderIdToXml() + "\n";
  xml = std::accumulate(datas_.begin(), datas_.end(), xml, [level, this](auto base, const feed_t& data){ return base + this->getSpaces(level + 1) + this->dataToXml(data) + "\n"; });
  xml = std::accumulate(nexts_.begin(), nexts_.end(), xml, [level](auto base, auto next){ return base + next->toXml(level + 1); });
  xml += getSpaces(level) + "</Node>\n";
  return xml;
}

std::string Version_node::getSpaces(int nb, const std::string& symbol)
{
  std::string res;
  for(int i = 0; i < nb; i++)
    res += symbol;
  return res;
}

std::string Version_node::dataToXml(const feed_t& data)
{
  std::string xml = "<Data action=\"" + std::string((data.action_ == action_add) ? "add" : "del") +
                    "\">" + data.from_ + "|" + data.prop_ + "|" + data.on_ +
                    "</Data>";
  return xml;

}

std::string Version_node::orderIdToXml()
{
  std::string xml = "<Order>" + std::to_string(order_id_) + "</Order>";
  return xml;
}

} // namespace ontologenius
