#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/interface/InterfaceParams.h"
#include "ontologenius/interface/RosInterface.h"

namespace ontologenius {

  bool RosInterface::classIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                      compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
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
          set_res_index = onto_->classes_.getDown(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getUp")
          set_res_index = onto_->classes_.getUp(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getDisjoint")
          set_res_index = onto_->classes_.getDisjoint(params.main_index);
        else if(req->action == "getName")
        {
          auto tmp = onto_->classes_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->string_values = onto_->classes_.getNames(params.main_index, params.take_id);
        else if(req->action == "getEveryNames")
          res->string_values = onto_->classes_.getEveryNames(params.main_index, params.take_id);
        else if(req->action == "getRelationFrom")
          set_res_index = onto_->classes_.getRelationFrom(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getRelatedFrom")
          set_res_index = onto_->classes_.getRelatedFrom(params.main_index);
        else if(req->action == "getRelationOn")
          set_res_index = onto_->classes_.getRelationOn(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getRelatedOn")
          set_res_index = onto_->classes_.getRelatedOn(params.main_index);
        else if(req->action == "getRelationWith")
          set_res_index = onto_->classes_.getRelationWith(params.main_index);
        else if(req->action == "getRelatedWith")
          set_res_index = onto_->classes_.getRelatedWith(params.main_index);
        else if(req->action == "getOn")
          set_res_index = onto_->classes_.getOn(params.main_index, params.optional_index);
        else if(req->action == "getFrom")
          set_res_index = onto_->classes_.getFrom(params.main_index, params.optional_index);
        else if(req->action == "getWith")
          set_res_index = onto_->classes_.getWith(params.main_index, params.optional_index, static_cast<int>(params.depth));
        else if(req->action == "getDomainOf")
          set_res_index = onto_->classes_.getDomainOf(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getRangeOf")
          set_res_index = onto_->classes_.getRangeOf(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "find")
          set2vector(onto_->classes_.find<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findSub")
          set2vector(onto_->classes_.findSub<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findRegex")
          set2vector(onto_->classes_.findRegex<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set2vector(onto_->classes_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->classes_.findFuzzy(params(), params.take_id), res->string_values);
        }
        else if(req->action == "getComments")
          res->string_values = onto_->classes_.getComments(params.main_index);
        else if(req->action == "exist")
        {
          if(onto_->classes_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
        }
        else if(req->action == "getAll")
          res->index_values = onto_->classes_.getAllIndex();
        else
          res->code = UNKNOW_ACTION;

        if(params.selector_index != 0)
        {
          if((req->action == "getUp") || (req->action == "getDown") ||
             (req->action == "getDisjoint") || (req->action == "getOn") ||
             (req->action == "getFrom"))
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
          else if((req->action == "getRelationFrom") || (req->action == "getRelationOn") || (req->action == "getWith") ||
                  (req->action == "getDomainOf") || (req->action == "getRangeOf"))
            set_res_index = onto_->object_properties_.select(set_res_index, params.selector_index);
        }

        if(res->index_values.empty())
          set2vector(set_res_index, res->index_values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::objectPropertyIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                               compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
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
          set_res_index = onto_->object_properties_.getDown(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getUp")
          set_res_index = onto_->object_properties_.getUp(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getDisjoint")
          set_res_index = onto_->object_properties_.getDisjoint(params.main_index);
        else if(req->action == "getInverse")
          set_res_index = onto_->object_properties_.getInverse(params.main_index);
        else if(req->action == "getDomain")
          set_res_index = onto_->object_properties_.getDomain(params.main_index, params.depth);
        else if(req->action == "getRange")
          set_res_index = onto_->object_properties_.getRange(params.main_index, params.depth);
        else if(req->action == "getName")
        {
          auto tmp = onto_->object_properties_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->string_values = onto_->object_properties_.getNames(params.main_index, params.take_id);
        else if(req->action == "getEveryNames")
          res->string_values = onto_->object_properties_.getEveryNames(params.main_index, params.take_id);
        else if(req->action == "find")
          set2vector(onto_->object_properties_.find<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findSub")
          set2vector(onto_->object_properties_.findSub<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findRegex")
          set2vector(onto_->object_properties_.findRegex<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set2vector(onto_->object_properties_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->object_properties_.findFuzzy(params(), params.take_id), res->string_values);
        }
        else if(req->action == "getComments")
          res->string_values = onto_->object_properties_.getComments(params.main_index);
        else if(req->action == "exist")
        {
          if(onto_->object_properties_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
        }
        else if(req->action == "getAll")
          res->index_values = onto_->object_properties_.getAllIndex();
        else
          res->code = UNKNOW_ACTION;

        if(params.selector_index != 0)
        {
          if((req->action == "getUp") || (req->action == "getDown") ||
             (req->action == "getDisjoint") || (req->action == "getInverse"))
            set_res_index = onto_->object_properties_.select(set_res_index, params.selector_index);
          else if((req->action == "getDomain") || (req->action == "getRange"))
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
        }

        if(res->index_values.empty())
          set2vector(set_res_index, res->index_values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::dataPropertyIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                             compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
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
          set_res_index = onto_->data_properties_.getDown(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getUp")
          set_res_index = onto_->data_properties_.getUp(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getDisjoint")
          set_res_index = onto_->data_properties_.getDisjoint(params.main_index);
        else if(req->action == "getDomain")
          set_res_index = onto_->data_properties_.getDomain(params.main_index, params.depth);
        else if(req->action == "getRange")
          set2vector(onto_->data_properties_.getRange(params.main_index), res->index_values);
        else if(req->action == "getName")
        {
          auto tmp = onto_->data_properties_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->string_values = onto_->data_properties_.getNames(params.main_index, params.take_id);
        else if(req->action == "getEveryNames")
          res->string_values = onto_->data_properties_.getEveryNames(params.main_index, params.take_id);
        else if(req->action == "find")
          set2vector(onto_->data_properties_.find<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findSub")
          set2vector(onto_->data_properties_.findSub<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findRegex")
          set2vector(onto_->data_properties_.findRegex<index_t>(params(), params.take_id), res->index_values);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set2vector(onto_->data_properties_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->data_properties_.findFuzzy(params(), params.take_id), res->string_values);
        }
        else if(req->action == "getComments")
          res->string_values = onto_->data_properties_.getComments(params.main_index);
        else if(req->action == "exist")
        {
          if(onto_->data_properties_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
        }
        else if(req->action == "getAll")
          res->index_values = onto_->data_properties_.getAllIndex();
        else
          res->code = UNKNOW_ACTION;

        if(params.selector_index != 0)
        {
          if((req->action == "getUp") || (req->action == "getDown") || (req->action == "getDisjoint"))
            set_res_index = onto_->data_properties_.select(set_res_index, params.selector_index);
          else if(req->action == "getDomain")
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
        }

        if(res->index_values.empty())
          set2vector(set_res_index, res->index_values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::individualIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Request>& req,
                                           compat::onto_ros::ServiceWrapper<compat::OntologeniusIndexService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
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
          set_res_index = onto_->individuals_.getSame(params.main_index);
        else if(req->action == "getDistincts")
          set_res_index = onto_->individuals_.getDistincts(params.main_index);
        else if(req->action == "getRelationFrom")
          set_res_index = onto_->individuals_.getRelationFrom(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getRelatedFrom")
          set_res_index = onto_->individuals_.getRelatedFrom(params.main_index);
        else if(req->action == "getRelationOn")
          set_res_index = onto_->individuals_.getRelationOn(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getRelatedOn")
          set_res_index = onto_->individuals_.getRelatedOn(params.main_index);
        else if(req->action == "getRelationWith")
          set_res_index = onto_->individuals_.getRelationWith(params.main_index);
        else if(req->action == "getRelatedWith")
          set_res_index = onto_->individuals_.getRelatedWith(params.main_index);
        else if(req->action == "getUp")
          set_res_index = onto_->individuals_.getUp(params.main_index, static_cast<int>(params.depth), false);
        else if(req->action == "getOn")
          set_res_index = onto_->individuals_.getOn(params.main_index, params.optional_index);
        else if(req->action == "getFrom")
          set_res_index = onto_->individuals_.getFrom(params.main_index, params.optional_index);
        else if(req->action == "getWith")
          set_res_index = onto_->individuals_.getWith(params.main_index, params.optional_index, static_cast<int>(params.depth));
        else if(req->action == "getDomainOf")
          set_res_index = onto_->individuals_.getDomainOf(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getRangeOf")
          set_res_index = onto_->individuals_.getRangeOf(params.main_index, static_cast<int>(params.depth));
        else if(req->action == "getName")
        {
          auto tmp = onto_->individuals_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->string_values = onto_->individuals_.getNames(params.main_index, params.take_id);
        else if(req->action == "getEveryNames")
          res->string_values = onto_->individuals_.getEveryNames(params.main_index, params.take_id);
        else if(req->action == "find")
          set_res_index = onto_->individuals_.find<index_t>(params(), params.take_id);
        else if(req->action == "findSub")
          set_res_index = onto_->individuals_.findSub<index_t>(params(), params.take_id);
        else if(req->action == "findRegex")
          set_res_index = onto_->individuals_.findRegex<index_t>(params(), params.take_id);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set2vector(onto_->individuals_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->individuals_.findFuzzy(params(), params.take_id), res->string_values);
        }
        else if(req->action == "getComments")
          res->string_values = onto_->individuals_.getComments(params.main_index);
        else if(req->action == "getType")
          set_res_index = onto_->individuals_.getType(params.main_index);
        else if(req->action == "exist")
        {
          if(onto_->individuals_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
        }
        /*else if(req->action == "relationExists")
        {
          if(onto_->individuals_.relationExists(params()))
            res->values.push_back(params());
        }*/
        else if(req->action == "getAll")
          res->index_values = onto_->individuals_.getAllIndex();
        else if(req->action == "isInferred")
          res->string_values = onto_->individuals_.isInferredIndex(params()) ? std::vector<std::string>{params()} : std::vector<std::string>{""};
        else if(req->action == "getInferenceExplanation")
          res->string_values = onto_->individuals_.getInferenceExplanationIndex(params());
        else if(req->action == "getInferenceRule")
          res->string_values = {onto_->individuals_.getInferenceRuleIndex(params())};
        else
          res->code = UNKNOW_ACTION;

        if(params.selector_index != 0)
        {
          if(req->action == "getUp")
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
          else if((req->action == "getRelationFrom") || (req->action == "getRelationOn") || (req->action == "getWith") ||
                  (req->action == "getDomainOf") || (req->action == "getRangeOf"))
            set_res_index = onto_->object_properties_.select(set_res_index, params.selector_index);
          else if((req->action != "find") || (req->action != "findRegex") || (req->action != "findSub") || (req->action != "findFuzzy") ||
                  (req->action != "getFrom") || (req->action != "getOn") ||
                  (req->action != "getRelationWith") || (req->action != "getRelatedWith"))
            set_res_index = onto_->individuals_.select(set_res_index, params.selector_index);
        }

        if(res->index_values.empty())
          set2vector(set_res_index, res->index_values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::sparqlIndexHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlIndexService::Request>& req,
                                       compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlIndexService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      const std::pair<std::vector<std::string>, std::vector<std::vector<index_t>>> results = sparql_.runIndex(req->query);

      if(results.second.empty() == false)
        res->names = results.first;

      for(const auto& result : results.second)
      {
        compat::OntologeniusSparqlIndexResponse tmp;
        for(const auto& r : result)
          tmp.values.push_back(r);

        res->results.push_back(tmp);
      }

      res->error = sparql_.getError();

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

} // namespace ontologenius
