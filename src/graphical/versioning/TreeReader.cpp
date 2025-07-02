#include "ontologenius/graphical/versioning/TreeReader.h"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>
#include <tinyxml2.h>

namespace ontologenius {

  size_t Commit::global_width = 1;
  size_t Commit::global_height = 0;

  Commit::Commit(const std::string& id) : id_(id), order_(-1)
  {
    global_height++;
  }

  Commit::~Commit()
  {
    for(auto& next : nexts_)
      delete next;
  }

  void Commit::setOrderId(size_t order)
  {
    order_ = (int)order;
  }

  void Commit::insertData(const std::string& data)
  {
    datas_.push_back(data);
    global_height++;
  }

  void Commit::insertNext(Commit* next)
  {
    nexts_.push_back(next);
    if(nexts_.size() > 1)
      global_width++;
  }

  Commit* TreeReader::read(const std::string& file_name)
  {
    std::string response;
    std::string tmp;
    std::ifstream f(file_name);

    if(!f.is_open())
    {
      std::cout << "Fail to open : " << file_name << std::endl;
      return nullptr;
    }

    while(getline(f, tmp))
    {
      response += tmp;
    }

    tinyxml2::XMLDocument doc;
    doc.Parse((const char*)response.c_str());
    tinyxml2::XMLElement* xml = doc.FirstChildElement();

    return readNode(xml);
  }

  Commit* TreeReader::readNode(tinyxml2::XMLElement* elem)
  {
    if(elem == nullptr)
    {
      std::cout << "Fail to read the file" << std::endl;
      return nullptr;
    }

    return readNode(elem, nullptr);
  }

  Commit* TreeReader::readNode(tinyxml2::XMLElement* elem, Commit* prev)
  {
    std::string elem_value = elem->Value();
    if(elem_value == "Node")
    {
      const std::string id = getAttribute(elem, "id");
      if(id.empty() == false)
      {
        auto* current = new Commit(id);
        std::cout << "create commit " << id << std::endl;
        for(tinyxml2::XMLElement* sub_elem = elem->FirstChildElement(); sub_elem != nullptr; sub_elem = sub_elem->NextSiblingElement())
        {
          elem_value = sub_elem->Value();
          if(elem_value == "Data")
          {
            std::string data = "[" + getAttribute(sub_elem, "action") + "]";
            const char* value = sub_elem->GetText();
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
            const char* value = sub_elem->GetText();
            if(value != nullptr)
            {
              const size_t order = std::stoi(std::string(value));
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

  std::string TreeReader::getAttribute(tinyxml2::XMLElement* sub_elem, const std::string& attribute)
  {
    const char* sub_attr = sub_elem->Attribute(attribute.c_str());
    if(sub_attr != nullptr)
      return std::string(sub_attr);
    return "";
  }

} // namespace ontologenius
