#include "ontologenius/interface/RosInterface.h"
#include "ontologenius/interface/InterfaceParams.h"

#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

bool RosInterface::classIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                    compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
{
  return [this](auto&& req, auto&& res)
  {
    res->code = 0;

    if(onto_->isInit() == false)
      res->code = UNINIT;
    else
    {
      removeUselessSpace(req->action);
      InterfaceParams params;
      params.extractIndexParams(req->param);

      reasoner_mutex_.lock();
      reasoners_.runPreReasoners(query_origin_class, req->action, params());
      reasoner_mutex_.unlock();

      std::unordered_set<index_t> set_res_index;

      if(req->action == "getDown")
        set_res_index = onto_->class_graph_.getDown(params.main_index, params.depth);
      else if(req->action == "getUp")
        set_res_index = onto_->class_graph_.getUp(params.main_index, params.depth);
      else if(req->action == "getDisjoint")
        set_res_index = onto_->class_graph_.getDisjoint(params.main_index);
      else if(req->action == "getName")
      {
        auto tmp = onto_->class_graph_.getName(params.main_index, params.take_id);
        if(tmp != "")
          res->string_values.push_back(tmp);
      }
      else if(req->action == "getNames")
        res->string_values = onto_->class_graph_.getNames(params.main_index, params.take_id);
      else if(req->action == "getEveryNames")
        res->string_values = onto_->class_graph_.getEveryNames(params.main_index, params.take_id);
      else if(req->action == "getRelationFrom")
        set_res_index = onto_->class_graph_.getRelationFrom(params.main_index, params.depth);
      else if(req->action == "getRelatedFrom")
        set_res_index = onto_->class_graph_.getRelatedFrom(params.main_index);
      else if(req->action == "getRelationOn")
        set_res_index = onto_->class_graph_.getRelationOn(params.main_index, params.depth);
      else if(req->action == "getRelatedOn")
        set_res_index = onto_->class_graph_.getRelatedOn(params.main_index);
      else if(req->action == "getRelationWith")
        set_res_index = onto_->class_graph_.getRelationWith(params.main_index);
      else if(req->action == "getRelatedWith")
        set_res_index = onto_->class_graph_.getRelatedWith(params.main_index);
      else if(req->action == "getOn")
        set_res_index = onto_->class_graph_.getOn(params.main_index, params.optional_index);
      else if(req->action == "getFrom")
        set_res_index = onto_->class_graph_.getFrom(params.main_index, params.optional_index);
      else if(req->action == "getWith")
        set_res_index = onto_->class_graph_.getWith(params.main_index, params.optional_index, params.depth);
      else if(req->action == "getDomainOf")
        set_res_index = onto_->class_graph_.getDomainOf(params.main_index, params.depth);
      else if(req->action == "getRangeOf")
        set_res_index = onto_->class_graph_.getRangeOf(params.main_index, params.depth);
      else if(req->action == "find")
        set2vector(onto_->class_graph_.find<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findSub")
        set2vector(onto_->class_graph_.findSub<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findRegex")
        set2vector(onto_->class_graph_.findRegex<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findFuzzy")
      {
        if(params.threshold != -1)
          set2vector(onto_->class_graph_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
        else
          set2vector(onto_->class_graph_.findFuzzy(params(), params.take_id), res->string_values);
      }
      else if(req->action == "exist")
      {
        if(onto_->class_graph_.touch(params.main_index))
          res->index_values.push_back(params.main_index);
      }
      else if(req->action == "getAll")
        res->index_values = onto_->class_graph_.getAllIndex();
      else
        res->code = UNKNOW_ACTION;

      if(params.selector_index != 0)
      {
        if((req->action == "getUp") || (req->action == "getDown") ||
          (req->action == "getDisjoint") || (req->action == "getOn") ||
          (req->action == "getFrom"))
          set_res_index = onto_->class_graph_.select(set_res_index, params.selector_index);
        else if((req->action == "getRelationFrom") || (req->action == "getRelationOn") || (req->action == "getWith") ||
                (req->action == "getDomainOf") || (req->action == "getRangeOf"))
          set_res_index = onto_->object_property_graph_.select(set_res_index, params.selector_index);
      }

      if(res->index_values.size() == 0)
        set2vector(set_res_index, res->index_values);
    }

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

bool RosInterface::objectPropertyIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                             compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
{
  return [this](auto&& req, auto&& res)
  {
    res->code = 0;

    if(onto_->isInit() == false)
      res->code = UNINIT;
    else
    {
      removeUselessSpace(req->action);
      InterfaceParams params;
      params.extractIndexParams(req->param);

      reasoner_mutex_.lock();
      reasoners_.runPreReasoners(query_origin_object_property, req->action, params());
      reasoner_mutex_.unlock();

      std::unordered_set<index_t> set_res_index;

      if(req->action == "getDown")
        set_res_index = onto_->object_property_graph_.getDown(params.main_index, params.depth);
      else if(req->action == "getUp")
        set_res_index = onto_->object_property_graph_.getUp(params.main_index, params.depth);
      else if(req->action == "getDisjoint")
        set_res_index = onto_->object_property_graph_.getDisjoint(params.main_index);
      else if(req->action == "getInverse")
        set_res_index = onto_->object_property_graph_.getInverse(params.main_index);
      else if(req->action == "getDomain")
        set_res_index = onto_->object_property_graph_.getDomain(params.main_index, params.depth);
      else if(req->action == "getRange")
        set_res_index = onto_->object_property_graph_.getRange(params.main_index, params.depth);
      else if(req->action == "getName")
      {
        auto tmp = onto_->object_property_graph_.getName(params.main_index, params.take_id);
        if(tmp != "")
          res->string_values.push_back(tmp);
      }
      else if(req->action == "getNames")
        res->string_values = onto_->object_property_graph_.getNames(params.main_index, params.take_id);
      else if(req->action == "getEveryNames")
        res->string_values = onto_->object_property_graph_.getEveryNames(params.main_index, params.take_id);
      else if(req->action == "find")
        set2vector(onto_->object_property_graph_.find<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findSub")
        set2vector(onto_->object_property_graph_.findSub<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findRegex")
        set2vector(onto_->object_property_graph_.findRegex<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findFuzzy")
      {
        if(params.threshold != -1)
          set2vector(onto_->object_property_graph_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
        else
          set2vector(onto_->object_property_graph_.findFuzzy(params(), params.take_id), res->string_values);
      }
      else if(req->action == "exist")
      {
        if(onto_->object_property_graph_.touch(params.main_index))
          res->index_values.push_back(params.main_index);
      }
      else if(req->action == "getAll")
        res->index_values = onto_->object_property_graph_.getAllIndex();
      else
        res->code = UNKNOW_ACTION;

      if(params.selector_index != 0)
      {
        if((req->action == "getUp") || (req->action == "getDown") ||
          (req->action == "getDisjoint") || (req->action == "getInverse"))
          set_res_index = onto_->object_property_graph_.select(set_res_index, params.selector_index);
        else if((req->action == "getDomain") || (req->action == "getRange"))
          set_res_index = onto_->class_graph_.select(set_res_index, params.selector_index);
      }

      if(res->index_values.size() == 0)
        set2vector(set_res_index, res->index_values);
    }

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

bool RosInterface::dataPropertyIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                           compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
{
  return [this](auto&& req, auto&& res)
  {
    res->code = 0;

    if(onto_->isInit() == false)
      res->code = UNINIT;
    else
    {
      removeUselessSpace(req->action);
      InterfaceParams params;
      params.extractIndexParams(req->param);

      reasoner_mutex_.lock();
      reasoners_.runPreReasoners(query_origin_data_property, req->action, params());
      reasoner_mutex_.unlock();

      std::unordered_set<index_t> set_res_index;

      if(req->action == "getDown")
        set_res_index = onto_->data_property_graph_.getDown(params.main_index, params.depth);
      else if(req->action == "getUp")
        set_res_index = onto_->data_property_graph_.getUp(params.main_index, params.depth);
      else if(req->action == "getDisjoint")
        set_res_index = onto_->data_property_graph_.getDisjoint(params.main_index);
      else if(req->action == "getDomain")
        set_res_index = onto_->data_property_graph_.getDomain(params.main_index, params.depth);
      else if(req->action == "getRange")
        set2vector(onto_->data_property_graph_.getRange(params.main_index), res->index_values);
      else if(req->action == "getName")
      {
        auto tmp = onto_->data_property_graph_.getName(params.main_index, params.take_id);
        if(tmp != "")
          res->string_values.push_back(tmp);
      }
      else if(req->action == "getNames")
        res->string_values = onto_->data_property_graph_.getNames(params.main_index, params.take_id);
      else if(req->action == "getEveryNames")
        res->string_values = onto_->data_property_graph_.getEveryNames(params.main_index, params.take_id);
      else if(req->action == "find")
        set2vector(onto_->data_property_graph_.find<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findSub")
        set2vector(onto_->data_property_graph_.findSub<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findRegex")
        set2vector(onto_->data_property_graph_.findRegex<index_t>(params(), params.take_id), res->index_values);
      else if(req->action == "findFuzzy")
      {
        if(params.threshold != -1)
          set2vector(onto_->data_property_graph_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
        else
          set2vector(onto_->data_property_graph_.findFuzzy(params(), params.take_id), res->string_values);
      }
      else if(req->action == "exist")
      {
        if(onto_->data_property_graph_.touch(params.main_index))
          res->index_values.push_back(params.main_index);
      }
      else if(req->action == "getAll")
        res->index_values = onto_->data_property_graph_.getAllIndex();
      else
        res->code = UNKNOW_ACTION;

      if(params.selector_index != 0)
      {
        if((req->action == "getUp") || (req->action == "getDown") || (req->action == "getDisjoint"))
          set_res_index = onto_->data_property_graph_.select(set_res_index, params.selector_index);
        else if(req->action == "getDomain")
          set_res_index = onto_->class_graph_.select(set_res_index, params.selector_index);
      }

      if(res->index_values.size() == 0)
        set2vector(set_res_index, res->index_values);
    }

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

bool RosInterface::individualIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                         compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
{
  return [this](auto&& req, auto&& res)
  {
    res->code = 0;

    if(onto_->isInit() == false)
      res->code = UNINIT;
    else
    {
      removeUselessSpace(req->action);
      InterfaceParams params;
      params.extractIndexParams(req->param);

      reasoner_mutex_.lock();
      reasoners_.runPreReasoners(query_origin_individual, req->action, params());
      reasoner_mutex_.unlock();

      std::unordered_set<index_t> set_res_index;

      if(req->action == "getSame")
        set_res_index = onto_->individual_graph_.getSame(params.main_index);
      else if(req->action == "getDistincts")
        set_res_index = onto_->individual_graph_.getDistincts(params.main_index);
      else if(req->action == "getRelationFrom")
        set_res_index = onto_->individual_graph_.getRelationFrom(params.main_index, params.depth);
      else if(req->action == "getRelatedFrom")
        set_res_index = onto_->individual_graph_.getRelatedFrom(params.main_index);
      else if(req->action == "getRelationOn")
        set_res_index = onto_->individual_graph_.getRelationOn(params.main_index, params.depth);
      else if(req->action == "getRelatedOn")
        set_res_index = onto_->individual_graph_.getRelatedOn(params.main_index);
      else if(req->action == "getRelationWith")
        set_res_index = onto_->individual_graph_.getRelationWith(params.main_index);
      else if(req->action == "getRelatedWith")
        set_res_index = onto_->individual_graph_.getRelatedWith(params.main_index);
      else if(req->action == "getUp")
        set_res_index = onto_->individual_graph_.getUp(params.main_index, params.depth);
      else if(req->action == "getOn")
        set_res_index = onto_->individual_graph_.getOn(params.main_index, params.optional_index);
      else if(req->action == "getFrom")
        set_res_index = onto_->individual_graph_.getFrom(params.main_index, params.optional_index);
      else if(req->action == "getWith")
        set_res_index = onto_->individual_graph_.getWith(params.main_index, params.optional_index, params.depth);
      else if(req->action == "getDomainOf")
        set_res_index = onto_->individual_graph_.getDomainOf(params.main_index, params.depth);
      else if(req->action == "getRangeOf")
        set_res_index = onto_->individual_graph_.getRangeOf(params.main_index, params.depth);
      else if(req->action == "getName")
      {
        auto tmp = onto_->individual_graph_.getName(params.main_index, params.take_id);
        if(tmp != "")
          res->string_values.push_back(tmp);
      }
      else if(req->action == "getNames")
        res->string_values = onto_->individual_graph_.getNames(params.main_index, params.take_id);
      else if(req->action == "getEveryNames")
        res->string_values = onto_->individual_graph_.getEveryNames(params.main_index, params.take_id);
      else if(req->action == "find")
        set_res_index = onto_->individual_graph_.find<index_t>(params(), params.take_id);
      else if(req->action == "findSub")
        set_res_index = onto_->individual_graph_.findSub<index_t>(params(), params.take_id);
      else if(req->action == "findRegex")
        set_res_index = onto_->individual_graph_.findRegex<index_t>(params(), params.take_id);
      else if(req->action == "findFuzzy")
      {
        if(params.threshold != -1)
          set2vector(onto_->individual_graph_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
        else
          set2vector(onto_->individual_graph_.findFuzzy(params(), params.take_id), res->string_values);
      }
      else if(req->action == "getType")
        set_res_index = onto_->individual_graph_.getType(params.main_index);
      else if(req->action == "exist")
      {
        if(onto_->individual_graph_.touch(params.main_index))
          res->index_values.push_back(params.main_index);
      }
      /*else if(req->action == "relationExists")
      {
        if(onto_->individual_graph_.relationExists(params()))
          res->values.push_back(params());
      }*/
      else if(req->action == "getAll")
        res->index_values = onto_->individual_graph_.getAllIndex();
      else
        res->code = UNKNOW_ACTION;

      if(params.selector_index != 0)
      {
        if(req->action == "getUp")
          set_res_index = onto_->class_graph_.select(set_res_index, params.selector_index);
        else if((req->action == "getRelationFrom") || (req->action == "getRelationOn") || (req->action == "getWith") ||
                (req->action == "getDomainOf") || (req->action == "getRangeOf"))
          set_res_index = onto_->object_property_graph_.select(set_res_index, params.selector_index);
        else if((req->action != "find") || (req->action != "findRegex") || (req->action != "findSub") ||
                (req->action != "getFrom") || (req->action != "getOn"))
          set_res_index = onto_->individual_graph_.select(set_res_index, params.selector_index);
      }

      if(res->index_values.size() == 0)
        set2vector(set_res_index, res->index_values);
    }

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

bool RosInterface::sparqlIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlIndexService::Request>& req,
                                     compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlIndexService::Response>& res)
{
  return [this](auto&& req, auto&& res)
  {
    std::pair<std::vector<std::string>, std::vector<std::vector<index_t>>> results = sparql_.runIndex(req->query);

    if(results.second.size())
      res->names = results.first;

    for(auto& result : results.second)
    {
      compat::OntologeniusSparqlIndexResponse tmp;
      for(auto& r : result)
        tmp.values.push_back(r);
      
      res->results.push_back(tmp);
    }

    res->error = sparql_.getError();

    return true;
  }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
}

} // namespace ontologenius