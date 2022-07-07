#ifndef ONTOLOGENIUS_DATA_H
#define ONTOLOGENIUS_DATA_H

#include <string>
#include <functional>

namespace ontologenius {

struct data_t
{
  std::string value_;
  std::string type_;
  //size_t hash_;

  data_t(const std::string& type, const std::string& value) : value_(value), type_(type)
  {}

  data_t(const std::string& value)
  {
    set(value);
  }

  data_t() {}

  std::string getNs() const
  {
    if((type_ == "real") || (type_ == "rational"))
      return "http://www.w3.org/2002/07/owl";
    else if((type_ == "PlainLiteral") || (type_ == "XMLLiteral"))
      return "http://www.w3.org/1999/02/22-rdf-syntax-ns";
    else if(type_ == "Literal")
      return "http://www.w3.org/2000/01/rdf-schema";
    else
      return "http://www.w3.org/2002/07/xsd"; //http://www.w3.org/2001/XMLSchema
  }

  std::string toString() const {return( type_ + "#" + value_); }
  void set(std::string value)
  {
    type_ = value.substr(0,value.find("#"));
    value_ = value.substr(value.find("#")+1);
  }

  bool operator==(const data_t& other) const
  {
    return ((type_ == other.type_) && (value_ == other.value_));
  }

  bool operator!=(const data_t& other) const
  {
    return ((type_ != other.type_) || (value_ != other.value_));
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATA_H
