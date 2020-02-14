#ifndef ONTOLOGENIUS_DATA_H
#define ONTOLOGENIUS_DATA_H

#include <string>

namespace ontologenius {

struct data_t
{
  std::string value_;
  std::string type_;

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
    auto pose = value.find("#");
    if(pose != std::string::npos)
    {
      type_ = value.substr(0,pose);
      value_ = value.substr(pose+1);
    }
  }

  bool operator==(const data_t& other)
  {
    if((value_ == other.value_) &&
      (type_ == other.type_))
      return true;
    else
      return false;
  }

  bool operator!=(const data_t& other)
  {
    if((value_ != other.value_) ||
      (type_ != other.type_))
      return true;
    else
      return false;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATA_H
