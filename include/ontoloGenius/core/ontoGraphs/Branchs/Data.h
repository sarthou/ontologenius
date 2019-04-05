#ifndef ONTOLOGENIUS_DATA_H
#define ONTOLOGENIUS_DATA_H

#include <string>

namespace ontologenius {

struct data_t
{
  std::string value_;
  std::string type_;

  std::string getNs()
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

  std::string toString() {return( type_ + ":" + value_); }
  void set(std::string value)
  {
    type_ = value.substr(0,value.find(":"));
    value_ = value.substr(value.find(":")+1);
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATA_H
