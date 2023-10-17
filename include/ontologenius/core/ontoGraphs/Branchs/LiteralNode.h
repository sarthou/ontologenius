#ifndef ONTOLOGENIUS_LITERALNODE_H
#define ONTOLOGENIUS_LITERALNODE_H

#include <string>
#include <functional>

#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"

namespace ontologenius {

class LiteralNode
{
public:
  std::string value_;
  std::string type_;
  
  static WordTable table_;

  LiteralNode(const std::string& type, const std::string& value) : value_(value), type_(type)
  {
    index_ = table_.add(type + "#" + value);
  }

  explicit LiteralNode(const std::string& value)
  {
    index_ = table_.add(value);
    set(value);
  }

  std::string getNs() const
  {
    if((type_ == "real") || (type_ == "rational"))
      return "http://www.w3.org/2002/07/owl";
    else if((type_ == "PlainLiteral") || (type_ == "XMLLiteral"))
      return "http://www.w3.org/1999/02/22-rdf-syntax-ns";
    else if(type_ == "Literal")
      return "http://www.w3.org/2000/01/rdf-schema";
    else if((type_ == "boolean") || (type_ == "string") || (type_ == "double") || (type_ == "integer"))
      return "http://www.w3.org/2001/XMLSchema";
    else
      return "http://www.w3.org/2002/07/xsd"; //http://www.w3.org/2001/XMLSchema
  }

  std::string toString() const { return( type_ + "#" + value_); }
  const std::string& value() const { return table_[index_]; }
  index_t get() const { return -index_; }
  void set(const std::string& value)
  {
    size_t pose = value.find("#");
    type_ = value.substr(0, pose);
    value_ = value.substr(pose + 1);
  }

  bool operator==(const LiteralNode& other) const
  {
    return ((type_ == other.type_) && (value_ == other.value_));
  }

  bool operator!=(const LiteralNode& other) const
  {
    return ((type_ != other.type_) || (value_ != other.value_));
  }

private:
  index_t index_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_LITERALNODE_H
