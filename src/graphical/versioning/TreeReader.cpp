#include "ontologenius/graphical/versioning/TreeReader.h"

#include <fstream>

namespace ontologenius {

size_t commit_t::global_width = 1;
size_t commit_t::global_height = 0;

commit_t::commit_t(const std::string& id)
{
  id_ = id;
  order_ = -1;
  global_height++;
}

commit_t::~commit_t()
{
  for(auto& next : nexts_)
    delete next;
}

void commit_t::setOrderId(size_t order)
{
  order_ = order;
}

void commit_t::insertData(const std::string& data)
{
  datas_.push_back(data);
  global_height++;
}

void commit_t::insertNext(commit_t* next)
{
  nexts_.push_back(next);
  if(nexts_.size() > 1)
    global_width++;
}

commit_t* TreeReader::read(const std::string& file_name)
{
  std::string response = "";
  std::string tmp = "";
  std::ifstream f(file_name);

  if(!f.is_open())
  {
    std::cout << "Fail to open : " << file_name << std::endl;
    return nullptr;
  }

  while(getline(f,tmp))
  {
    response += tmp;
  }

  TiXmlDocument doc;
  doc.Parse((const char*)response.c_str(), nullptr, TIXML_ENCODING_UTF8);
  TiXmlElement* xml = doc.FirstChildElement();

  return readNode(xml);
}

commit_t* TreeReader::readNode(TiXmlElement* elem)
{
  if(elem == nullptr)
  {
    std::cout << "Fail to read the file" << std::endl;
    return nullptr;
  }

  return readNode(elem, nullptr);
}

commit_t* TreeReader::readNode(TiXmlElement* elem, commit_t* prev)
{
  std::string elem_value = elem->Value();
  if(elem_value == "Node")
  {
    std::string id = getAttribute(elem, "id");
    if(id != "")
    {
      commit_t* current = new commit_t(id);
      std::cout << "create commit " << id << std::endl;
      for(TiXmlElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
      {
        elem_value = sub_elem->Value();
        if(elem_value == "Data")
        {
          std::string data = "[" + getAttribute(sub_elem, "action") + "]";
          const char* value;
          value = sub_elem->GetText();
          if(value != nullptr)
          {
            data += std::string(value);
            std::cout << "add data " << data << std::endl;
            current->insertData(data);
          }
        }
        else if(elem_value == "Node")
          readNode(sub_elem, current);
        else if(elem_value == "Order")
        {
          const char* value;
          value = sub_elem->GetText();
          if(value != nullptr)
          {
            size_t order = std::stoi(std::string(value));
            current->setOrderId(order);
          }
        }
      }

      if(prev != nullptr)
        prev->insertNext(current);
      return current;
    }
    else
      return nullptr;
  }
  else
    return nullptr;
}

std::string TreeReader::getAttribute(TiXmlElement* sub_elem, const std::string& attribute)
{
  const char* subAttr;
  subAttr = sub_elem->Attribute(attribute.c_str());
  if(subAttr != NULL)
    return std::string(subAttr);
  return "";
}

} // namespace ontologenius
