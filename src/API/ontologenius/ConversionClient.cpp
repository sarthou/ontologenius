#include "ontologenius/API/ontologenius/ConversionClient.h"

#include <iostream>

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

using namespace ontologenius;
using OntologeniusConversionRequestAlias = compat::RawRequestType<ontologenius::compat::OntologeniusConversion>;

ConversionClient::ConversionClient(const std::string& name) : name_((name == "") ? "/ontologenius/conversion" : "/ontologenius/conversion/" + name),
                                                              verbose_(false),
                                                              client_(name_)
{}

std::vector<std::string> ConversionClient::individualsIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, OntologeniusConversionRequestAlias::INDIVIDUALS);
}

std::vector<std::string> ConversionClient::classesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, OntologeniusConversionRequestAlias::CLASSES);
}

std::vector<std::string> ConversionClient::dataPropertiesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, OntologeniusConversionRequestAlias::DATA_PROPERTIES);
}

std::vector<std::string> ConversionClient::objectPropertiesIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, OntologeniusConversionRequestAlias::OBJECT_PROPERTIES);
}

std::vector<std::string> ConversionClient::literalsIndex2Id(const std::vector<int64_t>& indexes)
{
  return index2Id(indexes, OntologeniusConversionRequestAlias::LITERAL);
}

std::string ConversionClient::individualsIndex2Id(int64_t index)
{
  return index2Id(index, OntologeniusConversionRequestAlias::INDIVIDUALS);
}

std::string ConversionClient::classesIndex2Id(int64_t index)
{
  return index2Id(index, OntologeniusConversionRequestAlias::CLASSES);
}

std::string ConversionClient::dataPropertiesIndex2Id(int64_t index)
{
  return index2Id(index, OntologeniusConversionRequestAlias::DATA_PROPERTIES);
}

std::string ConversionClient::objectPropertiesIndex2Id(int64_t index)
{
  return index2Id(index, OntologeniusConversionRequestAlias::OBJECT_PROPERTIES);
}

std::string ConversionClient::literalsIndex2Id(int64_t index)
{
  return index2Id(index, OntologeniusConversionRequestAlias::LITERAL);
}

std::vector<int64_t> ConversionClient::individualsId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, OntologeniusConversionRequestAlias::INDIVIDUALS);
}

std::vector<int64_t> ConversionClient::classesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, OntologeniusConversionRequestAlias::CLASSES);
}

std::vector<int64_t> ConversionClient::dataPropertiesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, OntologeniusConversionRequestAlias::DATA_PROPERTIES);
}

std::vector<int64_t> ConversionClient::objectPropertiesId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, OntologeniusConversionRequestAlias::OBJECT_PROPERTIES);
}

std::vector<int64_t> ConversionClient::literalsId2Index(const std::vector<std::string>& ids)
{
  return Id2Index(ids, OntologeniusConversionRequestAlias::LITERAL);
}

int64_t ConversionClient::individualsId2Index(const std::string& id)
{
  return Id2Index(id, OntologeniusConversionRequestAlias::INDIVIDUALS);
}

int64_t ConversionClient::classesId2Index(const std::string& id)
{
  return Id2Index(id, OntologeniusConversionRequestAlias::CLASSES);
}

int64_t ConversionClient::dataPropertiesId2Index(const std::string& id)
{
  return Id2Index(id, OntologeniusConversionRequestAlias::DATA_PROPERTIES);
}

int64_t ConversionClient::objectPropertiesId2Index(const std::string& id)
{
  return Id2Index(id, OntologeniusConversionRequestAlias::OBJECT_PROPERTIES);
}

int64_t ConversionClient::literalsId2Index(const std::string& id)
{
  return Id2Index(id, OntologeniusConversionRequestAlias::LITERAL);
}

std::vector<std::string> ConversionClient::index2Id(const std::vector<int64_t>& indexes, int8_t source)
{
  auto req = ontologenius::compat::make_request<ontologenius::compat::OntologeniusConversion>();
  auto res = ontologenius::compat::make_response<ontologenius::compat::OntologeniusConversion>();

  [source, indexes](auto&& req)
  {
    req->source = source;
    req->values_int = indexes;
  }(ontologenius::compat::onto_ros::getServicePointer(req));

  if (call(req, res))
    return [](auto&& res)
    {
      return res->values_str;
    }(ontologenius::compat::onto_ros::getServicePointer(res));
  else
      return {};
}

std::string ConversionClient::index2Id(int64_t index, int8_t source)
{
  return index2Id(std::vector<int64_t>{ index }, source)[0];
}

std::vector<int64_t> ConversionClient::Id2Index(const std::vector<std::string>& ids, int8_t source)
{
  auto req = ontologenius::compat::make_request<ontologenius::compat::OntologeniusConversion>();
  auto res = ontologenius::compat::make_response<ontologenius::compat::OntologeniusConversion>();

  [source, ids](auto&& req)
  {
    req->source = source;
    req->values_str = ids;
  }(ontologenius::compat::onto_ros::getServicePointer(req));

  if (call(req, res))
    return [](auto&& res)
    {
      return res->values_int;
    }(ontologenius::compat::onto_ros::getServicePointer(res));
  else
      return {};
}

int64_t ConversionClient::Id2Index(const std::string& id, int8_t source)
{
  return Id2Index(std::vector<std::string>{ id }, source)[0];
}

bool ConversionClient::call(compat::RequestType<compat::OntologeniusConversion> &req,
                            compat::ResponseType<compat::OntologeniusConversion> &res)
{
  using ResultTy = typename decltype(client_)::Status;

  switch (client_.call(req, res))
  {
    case ResultTy::SUCCESSFUL_WITH_RETRIES:
      [[fallthrough]];
    case ResultTy::SUCCESSFUL:
    {
      return true;
    }
    case ResultTy::FAILURE:
    {
      if(verbose_)
        std::cout << COLOR_RED << "Failed to call ontologenius/" << name_ << COLOR_OFF << std::endl;
      [[fallthrough]];
    }
    default:
      return false;
  }
}

} // namespace onto
