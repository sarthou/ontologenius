#include "ontologenius/API/ontologenius/ConversionClient.h"

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE  "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN   "\x1B[1;92m"
#endif

namespace onto {

ConversionClient::ConversionClient(const std::string& name) : name_((name == "") ? "/ontologenius/conversion" : "/ontologenius/conversion/" + name),
                                                              verbose_(false),
                                                              client_(n_.serviceClient<ontologenius::OntologeniusConversion>(name_, true))
{}

std::vector<std::string> ConversionClient::individualsIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConversionRequest::INDIVIDUALS);
}

std::vector<std::string> ConversionClient::classesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConversionRequest::CLASSES);
}

std::vector<std::string> ConversionClient::dataPropertiesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConversionRequest::DATA_PROPERTIES);
}

std::vector<std::string> ConversionClient::objectPropertiesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConversionRequest::OBJECT_PROPERTIES);
}

std::vector<std::string> ConversionClient::literalsIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConversionRequest::LITERAL);
}

std::string ConversionClient::individualsIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConversionRequest::INDIVIDUALS);
}

std::string ConversionClient::classesIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConversionRequest::CLASSES);
}

std::string ConversionClient::dataPropertiesIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConversionRequest::DATA_PROPERTIES);
}

std::string ConversionClient::objectPropertiesIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConversionRequest::OBJECT_PROPERTIES);
}

std::string ConversionClient::literalsIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConversionRequest::LITERAL);
}

std::vector<int64_t> ConversionClient::individualsId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConversionRequest::INDIVIDUALS);
}

std::vector<int64_t> ConversionClient::classesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConversionRequest::CLASSES);
}

std::vector<int64_t> ConversionClient::dataPropertiesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConversionRequest::DATA_PROPERTIES);
}

std::vector<int64_t> ConversionClient::objectPropertiesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConversionRequest::OBJECT_PROPERTIES);
}

std::vector<int64_t> ConversionClient::literalsId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConversionRequest::LITERAL);
}

int64_t ConversionClient::individualsId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConversionRequest::INDIVIDUALS);
}

int64_t ConversionClient::classesId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConversionRequest::CLASSES);
}

int64_t ConversionClient::dataPropertiesId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConversionRequest::DATA_PROPERTIES);
}

int64_t ConversionClient::objectPropertiesId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConversionRequest::OBJECT_PROPERTIES);
}

int64_t ConversionClient::literalsId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConversionRequest::LITERAL);
}

std::vector<std::string> ConversionClient::index2Id(const std::vector<int64_t>& indexes, int8_t source)
{
  ontologenius::OntologeniusConversion srv;
  srv.request.source = source;
  srv.request.values_int = indexes;
  if(call(srv))
    return srv.response.values_str;
  else
    return {};
}

std::string ConversionClient::index2Id(int64_t index, int8_t source)
{
  ontologenius::OntologeniusConversion srv;
  srv.request.source = source;
  srv.request.values_int = {index};
  if(call(srv))
    return srv.response.values_str.front();
  else
    return "";
}

std::vector<int64_t> ConversionClient::Id2Index(const std::vector<std::string>& ids, int8_t source)
{
  ontologenius::OntologeniusConversion srv;
  srv.request.source = source;
  srv.request.values_str = ids;
  if(call(srv))
    return srv.response.values_int;
  else
    return {};
}

int64_t ConversionClient::Id2Index(const std::string& id, int8_t source)
{
  ontologenius::OntologeniusConversion srv;
  srv.request.source = source;
  srv.request.values_str = {id};
  if(call(srv))
    return srv.response.values_int.front();
  else
    return {};
}

bool ConversionClient::call(ontologenius::OntologeniusConversion& srv)
{
  if(client_.call(srv))
    return true;
  else
  {
    if(verbose_)
      std::cout << COLOR_ORANGE << "Failure to call " << name_ << COLOR_OFF << std::endl;
    client_ = n_.serviceClient<ontologenius::OntologeniusConversion>(name_, true);
    if(client_.call(srv))
    {
      if(verbose_)
        std::cout << COLOR_GREEN << "Restored " << name_ << COLOR_OFF << std::endl;
      return true;
    }
    else
    {
      if(verbose_)
        std::cout << COLOR_RED << "Failure of service restoration" << COLOR_OFF << std::endl;
      return false;
    }
  }
}

} // namespace onto
