#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/interface/InterfaceParams.h"
#include "ontologenius/interface/RosInterface.h"
#include "ontologenius/utils/String.h"

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
        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getDown"_act:
          set_res_index = onto_->classes_.getDown(params.main_index, static_cast<int>(params.depth));
          break;
        case "getUp"_act:
          set_res_index = onto_->classes_.getUp(params.main_index, static_cast<int>(params.depth));
          break;
        case "getDisjoint"_act:
          set_res_index = onto_->classes_.getDisjoint(params.main_index);
          break;
        case "getName"_act:
        {
          auto tmp = onto_->classes_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->string_values = onto_->classes_.getNames(params.main_index, params.take_id);
          break;
        case "getEveryNames"_act:
          res->string_values = onto_->classes_.getEveryNames(params.main_index, params.take_id);
          break;
        case "getRelationFrom"_act:
          set_res_index = onto_->classes_.getRelationFrom(params.main_index, static_cast<int>(params.depth));
          break;
        case "getRelatedFrom"_act:
          set_res_index = onto_->classes_.getRelatedFrom(params.main_index);
          break;
        case "getRelationOn"_act:
          set_res_index = onto_->classes_.getRelationOn(params.main_index, static_cast<int>(params.depth));
          break;
        case "getRelatedOn"_act:
          set_res_index = onto_->classes_.getRelatedOn(params.main_index);
          break;
        case "getRelationWith"_act:
          set_res_index = onto_->classes_.getRelationWith(params.main_index);
          break;
        case "getRelatedWith"_act:
          set_res_index = onto_->classes_.getRelatedWith(params.main_index);
          break;
        case "getOn"_act:
          set_res_index = onto_->classes_.getOn(params.main_index, params.optional_index);
          break;
        case "getFrom"_act:
          set_res_index = onto_->classes_.getFrom(params.main_index, params.optional_index);
          break;
        case "getWith"_act:
          set_res_index = onto_->classes_.getWith(params.main_index, params.optional_index, static_cast<int>(params.depth));
          break;
        case "getDomainOf"_act:
          set_res_index = onto_->classes_.getDomainOf(params.main_index, static_cast<int>(params.depth));
          break;
        case "getRangeOf"_act:
          set_res_index = onto_->classes_.getRangeOf(params.main_index, static_cast<int>(params.depth));
          break;
        case "find"_act:
          set2vector(onto_->classes_.find<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findSub"_act:
          set2vector(onto_->classes_.findSub<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findRegex"_act:
          set2vector(onto_->classes_.findRegex<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set2vector(onto_->classes_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->classes_.findFuzzy(params(), params.take_id), res->string_values);
          break;
        case "getComments"_act:
          res->string_values = onto_->classes_.getComments(params.main_index);
          break;
        case "exist"_act:
          if(onto_->classes_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
          break;
        case "getAll"_act:
          res->index_values = onto_->classes_.getAllIndex();
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector_index != 0)
        {
          switch(action_hash)
          {
          case "getUp"_act: [[fallthrough]];
          case "getDown"_act: [[fallthrough]];
          case "getDisjoint"_act: [[fallthrough]];
          case "getOn"_act: [[fallthrough]];
          case "getFrom"_act:
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
            break;
          case "getRelationFrom"_act: [[fallthrough]];
          case "getRelationOn"_act: [[fallthrough]];
          case "getWith"_act: [[fallthrough]];
          case "getDomainOf"_act: [[fallthrough]];
          case "getRangeOf"_act:
            set_res_index = onto_->object_properties_.select(set_res_index, params.selector_index);
            break;
          default:
            break;
          }
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
        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getDown"_act:
          set_res_index = onto_->object_properties_.getDown(params.main_index, static_cast<int>(params.depth));
          break;
        case "getUp"_act:
          set_res_index = onto_->object_properties_.getUp(params.main_index, static_cast<int>(params.depth));
          break;
        case "getDisjoint"_act:
          set_res_index = onto_->object_properties_.getDisjoint(params.main_index);
          break;
        case "getInverse"_act:
          set_res_index = onto_->object_properties_.getInverse(params.main_index);
          break;
        case "getDomain"_act:
          set_res_index = onto_->object_properties_.getDomain(params.main_index, params.depth);
          break;
        case "getRange"_act:
          set_res_index = onto_->object_properties_.getRange(params.main_index, params.depth);
          break;
        case "getName"_act:
        {
          auto tmp = onto_->object_properties_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->string_values = onto_->object_properties_.getNames(params.main_index, params.take_id);
          break;
        case "getEveryNames"_act:
          res->string_values = onto_->object_properties_.getEveryNames(params.main_index, params.take_id);
          break;
        case "find"_act:
          set2vector(onto_->object_properties_.find<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findSub"_act:
          set2vector(onto_->object_properties_.findSub<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findRegex"_act:
          set2vector(onto_->object_properties_.findRegex<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set2vector(onto_->object_properties_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->object_properties_.findFuzzy(params(), params.take_id), res->string_values);
          break;
        case "getComments"_act:
          res->string_values = onto_->object_properties_.getComments(params.main_index);
          break;
        case "exist"_act:
          if(onto_->object_properties_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
          break;
        case "getAll"_act:
          res->index_values = onto_->object_properties_.getAllIndex();
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector_index != 0)
        {
          switch(action_hash)
          {
          case "getUp"_act: [[fallthrough]];
          case "getDown"_act: [[fallthrough]];
          case "getDisjoint"_act: [[fallthrough]];
          case "getInverse"_act:
            set_res_index = onto_->object_properties_.select(set_res_index, params.selector_index);
            break;
          case "getDomain"_act: [[fallthrough]];
          case "getRange"_act:
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
            break;
          default:
            break;
          }
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
        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getDown"_act:
          set_res_index = onto_->data_properties_.getDown(params.main_index, static_cast<int>(params.depth));
          break;
        case "getUp"_act:
          set_res_index = onto_->data_properties_.getUp(params.main_index, static_cast<int>(params.depth));
          break;
        case "getDisjoint"_act:
          set_res_index = onto_->data_properties_.getDisjoint(params.main_index);
          break;
        case "getDomain"_act:
          set_res_index = onto_->data_properties_.getDomain(params.main_index, params.depth);
          break;
        case "getRange"_act:
          set2vector(onto_->data_properties_.getRange(params.main_index), res->index_values);
          break;
        case "getName"_act:
        {
          auto tmp = onto_->data_properties_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->string_values = onto_->data_properties_.getNames(params.main_index, params.take_id);
          break;
        case "getEveryNames"_act:
          res->string_values = onto_->data_properties_.getEveryNames(params.main_index, params.take_id);
          break;
        case "find"_act:
          set2vector(onto_->data_properties_.find<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findSub"_act:
          set2vector(onto_->data_properties_.findSub<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findRegex"_act:
          set2vector(onto_->data_properties_.findRegex<index_t>(params(), params.take_id), res->index_values);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set2vector(onto_->data_properties_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->data_properties_.findFuzzy(params(), params.take_id), res->string_values);
          break;
        case "getComments"_act:
          res->string_values = onto_->data_properties_.getComments(params.main_index);
          break;
        case "exist"_act:
          if(onto_->data_properties_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
          break;
        case "getAll"_act:
          res->index_values = onto_->data_properties_.getAllIndex();
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector_index != 0)
        {
          switch(action_hash)
          {
          case "getUp"_act: [[fallthrough]];
          case "getDown"_act: [[fallthrough]];
          case "getDisjoint"_act:
            set_res_index = onto_->data_properties_.select(set_res_index, params.selector_index);
            break;
          case "getDomain"_act:
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
            break;
          default:
            break;
          }
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
        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getSame"_act:
          set_res_index = onto_->individuals_.getSame(params.main_index);
          break;
        case "getDistincts"_act:
          set_res_index = onto_->individuals_.getDistincts(params.main_index);
          break;
        case "getRelationFrom"_act:
          set_res_index = onto_->individuals_.getRelationFrom(params.main_index, static_cast<int>(params.depth));
          break;
        case "getRelatedFrom"_act:
          set_res_index = onto_->individuals_.getRelatedFrom(params.main_index);
          break;
        case "getRelationOn"_act:
          set_res_index = onto_->individuals_.getRelationOn(params.main_index, static_cast<int>(params.depth));
          break;
        case "getRelatedOn"_act:
          set_res_index = onto_->individuals_.getRelatedOn(params.main_index);
          break;
        case "getRelationWith"_act:
          set_res_index = onto_->individuals_.getRelationWith(params.main_index);
          break;
        case "getRelatedWith"_act:
          set_res_index = onto_->individuals_.getRelatedWith(params.main_index);
          break;
        case "getUp"_act:
          set_res_index = onto_->individuals_.getUp(params.main_index, static_cast<int>(params.depth), false);
          break;
        case "getOn"_act:
          set_res_index = onto_->individuals_.getOn(params.main_index, params.optional_index);
          break;
        case "getFrom"_act:
          set_res_index = onto_->individuals_.getFrom(params.main_index, params.optional_index);
          break;
        case "getWith"_act:
          set_res_index = onto_->individuals_.getWith(params.main_index, params.optional_index, static_cast<int>(params.depth));
          break;
        case "getDomainOf"_act:
          set_res_index = onto_->individuals_.getDomainOf(params.main_index, static_cast<int>(params.depth));
          break;
        case "getRangeOf"_act:
          set_res_index = onto_->individuals_.getRangeOf(params.main_index, static_cast<int>(params.depth));
          break;
        case "getName"_act:
        {
          auto tmp = onto_->individuals_.getName(params.main_index, params.take_id);
          if(tmp.empty() == false)
            res->string_values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->string_values = onto_->individuals_.getNames(params.main_index, params.take_id);
          break;
        case "getEveryNames"_act:
          res->string_values = onto_->individuals_.getEveryNames(params.main_index, params.take_id);
          break;
        case "find"_act:
          set_res_index = onto_->individuals_.find<index_t>(params(), params.take_id);
          break;
        case "findSub"_act:
          set_res_index = onto_->individuals_.findSub<index_t>(params(), params.take_id);
          break;
        case "findRegex"_act:
          set_res_index = onto_->individuals_.findRegex<index_t>(params(), params.take_id);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set2vector(onto_->individuals_.findFuzzy(params(), params.take_id, params.threshold), res->string_values);
          else
            set2vector(onto_->individuals_.findFuzzy(params(), params.take_id), res->string_values);
          break;
        case "getComments"_act:
          res->string_values = onto_->individuals_.getComments(params.main_index);
          break;
        case "getType"_act:
          set_res_index = onto_->individuals_.getType(params.main_index);
          break;
        case "exist"_act:
          if(onto_->individuals_.touch(params.main_index))
            res->index_values.push_back(params.main_index);
          break;
        /*case "relationExists"_act:
          if(onto_->individuals_.relationExists(params()))
            res->values.push_back(params());
          break;*/
        case "getAll"_act:
          res->index_values = onto_->individuals_.getAllIndex();
          break;
        case "isInferred"_act:
          res->string_values = onto_->individuals_.isInferredIndex(params()) ? std::vector<std::string>{params()} : std::vector<std::string>{""};
          break;
        case "getInferenceExplanation"_act:
          res->string_values = onto_->individuals_.getInferenceExplanationIndex(params());
          break;
        case "getInferenceRule"_act:
          res->string_values = {onto_->individuals_.getInferenceRuleIndex(params())};
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector_index != 0)
        {
          switch(action_hash)
          {
          case "getUp"_act:
            set_res_index = onto_->classes_.select(set_res_index, params.selector_index);
            break;
          case "getRelationFrom"_act: [[fallthrough]];
          case "getRelationOn"_act: [[fallthrough]];
          case "getWith"_act: [[fallthrough]];
          case "getDomainOf"_act: [[fallthrough]];
          case "getRangeOf"_act:
            set_res_index = onto_->object_properties_.select(set_res_index, params.selector_index);
            break;
          case "find"_act: [[fallthrough]];
          case "findRegex"_act: [[fallthrough]];
          case "findSub"_act: [[fallthrough]];
          case "findFuzzy"_act: [[fallthrough]];
          case "getFrom"_act: [[fallthrough]];
          case "getOn"_act: [[fallthrough]];
          case "getRelationWith"_act: [[fallthrough]];
          case "getRelatedWith"_act:
            break;
          default:
            set_res_index = onto_->individuals_.select(set_res_index, params.selector_index);
            break;
          }
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
