#include "ontologenius/core/ontoGraphs/Graphs/LiteralGraph.h"

#include <cstddef>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"

namespace ontologenius {

  LiteralGraph::LiteralGraph()
  {
    findOrCreateType("real", "http://www.w3.org/2002/07/owl");
    findOrCreateType("rational", "http://www.w3.org/2002/07/owl");
    findOrCreateType("PlainLiteral", "http://www.w3.org/1999/02/22-rdf-syntax-ns");
    findOrCreateType("XMLLiteral", "http://www.w3.org/1999/02/22-rdf-syntax-ns");
    findOrCreateType("Literal", "http://www.w3.org/2000/01/rdf-schema");
    findOrCreateType("boolean", "http://www.w3.org/2001/XMLSchema");
    findOrCreateType("string", "http://www.w3.org/2001/XMLSchema");
    findOrCreateType("double", "http://www.w3.org/2001/XMLSchema");
    findOrCreateType("integer", "http://www.w3.org/2001/XMLSchema");
    findOrCreateType("nonNegativeInteger", "http://www.w3.org/2001/XMLSchema");
  }

  LiteralGraph::LiteralGraph(const LiteralGraph& other)
  {
    all_literals_.reserve(other.all_literals_.size());

    all_types_.reserve(other.all_types_.size());
    for(auto* type : other.all_types_)
      all_types_.push_back(new LiteralType(type->value(), type->getNamespace()));
  }

  void LiteralGraph::deepCopy(const LiteralGraph& other)
  {
    for(auto* literal : other.all_literals_)
    {
      auto* type = findOrCreateType(literal->type_->value(), literal->type_->getNamespace());
      all_literals_.push_back(new LiteralNode(type, literal->data()));
    }
    literal_container_.load(all_literals_);
  }

  LiteralNode* LiteralGraph::findOrCreate(const std::string& value)
  {
    auto* literal = literal_container_.find(value);

    if(literal == nullptr)
    {
      size_t pose = value.find('#');
      std::string type = value.substr(0, pose);
      std::string data = value.substr(pose + 1);

      auto* type_ptr = findOrCreateType(type);
      literal = new LiteralNode(type_ptr, data);
      literal_container_.insert(literal);
      all_literals_.push_back(literal);
    }
    return literal;
  }

  LiteralNode* LiteralGraph::findOrCreate(const std::string& type, const std::string& value)
  {
    return findOrCreate(type + "#" + value);
  }

  LiteralType* LiteralGraph::findOrCreateType(const std::string& value, const std::string& ns)
  {
    LiteralType* res = nullptr;
    for(auto* type : all_types_)
      if(type->value() == value)
      {
        res = type;
        break;
      }

    if(res == nullptr)
    {
      all_types_.push_back(new LiteralType(value, ns));
      res = all_types_.back();
    }

    return res;
  }

  LiteralNode* LiteralGraph::find(const std::string& type, const std::string& value)
  {
    return literal_container_.find(type + "#" + value);
  }

  LiteralNode* LiteralGraph::find(const std::string& value)
  {
    return literal_container_.find(value);
  }

  LiteralNode* LiteralGraph::find(index_t index)
  {
    return find(LiteralNode::table.get(-index));
  }

  std::vector<index_t> LiteralGraph::getIndexes(const std::vector<std::string>& values)
  {
    std::vector<index_t> res;
    res.reserve(values.size());
    for(const auto& value : values)
    {
      auto* literal = literal_container_.find(value);
      res.push_back((literal == nullptr) ? 0 : literal->get());
    }
    return res;
  }

  std::vector<std::string> LiteralGraph::getIdentifiers(const std::vector<index_t>& values)
  {
    std::vector<std::string> res;
    for(auto value : values)
    {
      if((value < 0) && (-value < static_cast<index_t>(all_literals_.size())))
        res.push_back(all_literals_[-value]->value());
      else
        res.emplace_back("");
    }
    return res;
  }

} // namespace ontologenius