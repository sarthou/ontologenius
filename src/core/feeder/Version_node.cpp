#include "ontologenius/core/feeder/Version_node.h"

namespace ontologenius {

Version_node::Version_node(Version_node* prev)
{
  prev_ = prev;
  if(prev_ != nullptr)
    prev_->addNext(this);
  id_ = "";
}

Version_node::Version_node(const std::string& id)
{
  id_ = id;
  prev_ = nullptr;
}

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
  for(auto& data : datas_)
    xml += getSpaces(level + 1) + dataToXml(data) + "\n";
  for(auto& next : nexts_)
    xml += next->toXml(level + 1);
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

std::string Version_node::dataToXml(feed_t data)
{
  std::string xml = "<Data action=\"" + std::string((data.action_ == action_add) ? "add" : "del") +
                    "\">" + data.from_ + "|" + data.prop_ + "|" + data.on_ +
                    "</Data>";
  return xml;

}

} // namespace ontologenius
