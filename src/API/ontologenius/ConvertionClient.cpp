#include "ontologenius/API/ontologenius/ConvertionClient.h"

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

ConvertionClient::ConvertionClient(ros::NodeHandle* n, const std::string& name) : name_((name == "") ? "/ontologenius/convertion" : "/ontologenius/convertion/" + name),
                                                                                  n_(n), verbose_(false),
                                                                                  client_(n->serviceClient<ontologenius::OntologeniusConvertion>(name_, true))
{}

std::vector<std::string> ConvertionClient::individualsIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConvertionRequest::INDIVIDUALS);
}

std::vector<std::string> ConvertionClient::classesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConvertionRequest::CLASSES);
}

std::vector<std::string> ConvertionClient::dataPropertiesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConvertionRequest::DATA_PROPERTIES);
}

std::vector<std::string> ConvertionClient::objectPropertiesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConvertionRequest::OBJECT_PROPERTIES);
}

std::vector<std::string> ConvertionClient::literalsIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, ontologenius::OntologeniusConvertionRequest::LITERAL);
}

std::string ConvertionClient::individualsIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConvertionRequest::INDIVIDUALS);
}

std::string ConvertionClient::classesIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConvertionRequest::CLASSES);
}

std::string ConvertionClient::dataPropertiesIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConvertionRequest::DATA_PROPERTIES);
}

std::string ConvertionClient::objectPropertiesIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConvertionRequest::OBJECT_PROPERTIES);
}

std::string ConvertionClient::literalsIndex2Id(int64_t index)
{
  return index2Id(index, ontologenius::OntologeniusConvertionRequest::LITERAL);
}

std::vector<int64_t> ConvertionClient::individualsId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConvertionRequest::INDIVIDUALS);
}

std::vector<int64_t> ConvertionClient::classesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConvertionRequest::CLASSES);
}

std::vector<int64_t> ConvertionClient::dataPropertiesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConvertionRequest::DATA_PROPERTIES);
}

std::vector<int64_t> ConvertionClient::objectPropertiesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConvertionRequest::OBJECT_PROPERTIES);
}

std::vector<int64_t> ConvertionClient::literalsId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, ontologenius::OntologeniusConvertionRequest::LITERAL);
}

int64_t ConvertionClient::individualsId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConvertionRequest::INDIVIDUALS);
}

int64_t ConvertionClient::classesId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConvertionRequest::CLASSES);
}

int64_t ConvertionClient::dataPropertiesId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConvertionRequest::DATA_PROPERTIES);
}

int64_t ConvertionClient::objectPropertiesId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConvertionRequest::OBJECT_PROPERTIES);
}

int64_t ConvertionClient::literalsId2Index(const std::string& id)
{
  return Id2Index(id, ontologenius::OntologeniusConvertionRequest::LITERAL);
}

std::vector<std::string> ConvertionClient::index2Id(const std::vector<int64_t>& indexes, int8_t source)
{
  ontologenius::OntologeniusConvertion srv;
  srv.request.source = source;
  srv.request.values_int = indexes;
  if(call(srv))
    return srv.response.values_str;
  else
    return {};
}

std::string ConvertionClient::index2Id(int64_t index, int8_t source)
{
  ontologenius::OntologeniusConvertion srv;
  srv.request.source = source;
  srv.request.values_int = {index};
  if(call(srv))
    return srv.response.values_str.front();
  else
    return "";
}

std::vector<int64_t> ConvertionClient::Id2Index(const std::vector<std::string>& ids, int8_t source)
{
  ontologenius::OntologeniusConvertion srv;
  srv.request.source = source;
  srv.request.values_str = ids;
  if(call(srv))
    return srv.response.values_int;
  else
    return {};
}

int64_t ConvertionClient::Id2Index(const std::string& id, int8_t source)
{
  ontologenius::OntologeniusConvertion srv;
  srv.request.source = source;
  srv.request.values_str = {id};
  if(call(srv))
    return srv.response.values_int.front();
  else
    return {};
}

bool ConvertionClient::call(ontologenius::OntologeniusConvertion& srv)
{
  if(client_.call(srv))
    return true;
  else
  {
    if(verbose_)
      std::cout << COLOR_ORANGE << "Failure to call " << name_ << COLOR_OFF << std::endl;
    client_ = n_->serviceClient<ontologenius::OntologeniusConvertion>(name_, true);
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
