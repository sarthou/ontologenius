#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/interface/InterfaceParams.h"
#include "ontologenius/interface/RosInterface.h"
#include "ontologenius/utils/String.h"

namespace ontologenius {

  bool RosInterface::classHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request>& req,
                                 compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      res->code = 0;

      if(onto_->isInit() == false)
        res->code = UNINIT;
      else
      {
        removeUselessSpace(req->action);
        InterfaceParams params;
        params.extractStringParams(req->param);

        reasoner_mutex_.lock();
        reasoners_.runPreReasoners(query_origin_class, req->action, params());
        reasoner_mutex_.unlock();

        std::unordered_set<std::string> set_res;
        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getDown"_act:
          set_res = onto_->classes_.getDown(params(), static_cast<int>(params.depth));
          break;
        case "getUp"_act:
          set_res = onto_->classes_.getUp(params(), static_cast<int>(params.depth));
          break;
        case "getDisjoint"_act:
          set_res = onto_->classes_.getDisjoint(params());
          break;
        case "getName"_act:
        {
          auto tmp = onto_->classes_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->values = onto_->classes_.getNames(params(), params.take_id);
          break;
        case "getEveryNames"_act:
          res->values = onto_->classes_.getEveryNames(params(), params.take_id);
          break;
        case "getRelationFrom"_act:
          set_res = onto_->classes_.getRelationFrom(params(), static_cast<int>(params.depth));
          break;
        case "getRelatedFrom"_act:
          set_res = onto_->classes_.getRelatedFrom(params());
          break;
        case "getRelationOn"_act:
          set_res = onto_->classes_.getRelationOn(params(), static_cast<int>(params.depth));
          break;
        case "getRelatedOn"_act:
          set_res = onto_->classes_.getRelatedOn(params());
          break;
        case "getRelationWith"_act:
          set_res = onto_->classes_.getRelationWith(params());
          break;
        case "getRelatedWith"_act:
          set_res = onto_->classes_.getRelatedWith(params());
          break;
        case "getOn"_act:
          set_res = onto_->classes_.getOn(params());
          break;
        case "getFrom"_act:
          set_res = onto_->classes_.getFrom(params());
          break;
        case "getWith"_act:
          set_res = onto_->classes_.getWith(params(), static_cast<int>(params.depth));
          break;
        case "getDomainOf"_act:
          set_res = onto_->classes_.getDomainOf(params(), static_cast<int>(params.depth));
          break;
        case "getRangeOf"_act:
          set_res = onto_->classes_.getRangeOf(params(), static_cast<int>(params.depth));
          break;
        case "find"_act:
          set2vector(onto_->classes_.find<std::string>(params(), params.take_id), res->values);
          break;
        case "findSub"_act:
          set2vector(onto_->classes_.findSub<std::string>(params(), params.take_id), res->values);
          break;
        case "findRegex"_act:
          set2vector(onto_->classes_.findRegex<std::string>(params(), params.take_id), res->values);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set2vector(onto_->classes_.findFuzzy(params(), params.take_id, params.threshold), res->values);
          else
            set2vector(onto_->classes_.findFuzzy(params(), params.take_id), res->values);
          break;
        case "exist"_act:
          if(onto_->classes_.touch(params()))
            res->values.push_back(params());
          break;
        case "getAll"_act:
          res->values = onto_->classes_.getAll();
          break;
        case "getComments"_act:
          res->values = onto_->classes_.getComments(params());
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector.empty() == false)
        {
          switch(action_hash)
          {
          case "getUp"_act: [[fallthrough]];
          case "getDown"_act: [[fallthrough]];
          case "getDisjoint"_act: [[fallthrough]];
          case "getOn"_act: [[fallthrough]];
          case "getFrom"_act:
            set_res = onto_->classes_.select(set_res, params.selector);
            break;
          case "getRelationFrom"_act: [[fallthrough]];
          case "getRelationOn"_act: [[fallthrough]];
          case "getWith"_act: [[fallthrough]];
          case "getDomainOf"_act: [[fallthrough]];
          case "getRangeOf"_act:
            set_res = onto_->object_properties_.select(set_res, params.selector);
            break;
          default:
            break;
          }
        }

        if(res->values.empty())
          set2vector(set_res, res->values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::objectPropertyHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request>& req,
                                          compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      res->code = 0;

      if(onto_->isInit() == false)
        res->code = UNINIT;
      else
      {
        removeUselessSpace(req->action);
        InterfaceParams params;
        params.extractStringParams(req->param);

        reasoner_mutex_.lock();
        reasoners_.runPreReasoners(query_origin_object_property, req->action, params());
        reasoner_mutex_.unlock();

        std::unordered_set<std::string> set_res;
        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getDown"_act:
          set_res = onto_->object_properties_.getDown(params(), static_cast<int>(params.depth));
          break;
        case "getUp"_act:
          set_res = onto_->object_properties_.getUp(params(), static_cast<int>(params.depth));
          break;
        case "getDisjoint"_act:
          set_res = onto_->object_properties_.getDisjoint(params());
          break;
        case "getInverse"_act:
          set_res = onto_->object_properties_.getInverse(params());
          break;
        case "getDomain"_act:
          set_res = onto_->object_properties_.getDomain(params(), params.depth);
          break;
        case "getRange"_act:
          set_res = onto_->object_properties_.getRange(params(), params.depth);
          break;
        case "getName"_act:
        {
          auto tmp = onto_->object_properties_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->values = onto_->object_properties_.getNames(params(), params.take_id);
          break;
        case "getEveryNames"_act:
          res->values = onto_->object_properties_.getEveryNames(params(), params.take_id);
          break;
        case "find"_act:
          set2vector(onto_->object_properties_.find<std::string>(params(), params.take_id), res->values);
          break;
        case "findSub"_act:
          set2vector(onto_->object_properties_.findSub<std::string>(params(), params.take_id), res->values);
          break;
        case "findRegex"_act:
          set2vector(onto_->object_properties_.findRegex<std::string>(params(), params.take_id), res->values);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set2vector(onto_->object_properties_.findFuzzy(params(), params.take_id, params.threshold), res->values);
          else
            set2vector(onto_->object_properties_.findFuzzy(params(), params.take_id), res->values);
          break;
        case "exist"_act:
          if(onto_->object_properties_.touch(params()))
            res->values.push_back(params());
          break;
        case "getAll"_act:
          res->values = onto_->object_properties_.getAll();
          break;
        case "getComments"_act:
          res->values = onto_->object_properties_.getComments(params());
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector.empty() == false)
        {
          switch(action_hash)
          {
          case "getUp"_act: [[fallthrough]];
          case "getDown"_act: [[fallthrough]];
          case "getDisjoint"_act: [[fallthrough]];
          case "getInverse"_act:
            set_res = onto_->object_properties_.select(set_res, params.selector);
            break;
          case "getDomain"_act: [[fallthrough]];
          case "getRange"_act:
            set_res = onto_->classes_.select(set_res, params.selector);
            break;
          default:
            break;
          }
        }

        if(res->values.empty())
          set2vector(set_res, res->values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::dataPropertyHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request>& req,
                                        compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      res->code = 0;

      if(onto_->isInit() == false)
        res->code = UNINIT;
      else
      {
        removeUselessSpace(req->action);
        InterfaceParams params;
        params.extractStringParams(req->param);

        reasoner_mutex_.lock();
        reasoners_.runPreReasoners(query_origin_data_property, req->action, params());
        reasoner_mutex_.unlock();

        std::unordered_set<std::string> set_res;
        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getDown"_act:
          set_res = onto_->data_properties_.getDown(params(), static_cast<int>(params.depth));
          break;
        case "getUp"_act:
          set_res = onto_->data_properties_.getUp(params(), static_cast<int>(params.depth));
          break;
        case "getDisjoint"_act:
          set_res = onto_->data_properties_.getDisjoint(params());
          break;
        case "getDomain"_act:
          set_res = onto_->data_properties_.getDomain(params(), params.depth);
          break;
        case "getRange"_act:
          set2vector(onto_->data_properties_.getRange(params()), res->values);
          break;
        case "getName"_act:
        {
          auto tmp = onto_->data_properties_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->values = onto_->data_properties_.getNames(params(), params.take_id);
          break;
        case "getEveryNames"_act:
          res->values = onto_->data_properties_.getEveryNames(params(), params.take_id);
          break;
        case "find"_act:
          set2vector(onto_->data_properties_.find<std::string>(params(), params.take_id), res->values);
          break;
        case "findSub"_act:
          set2vector(onto_->data_properties_.findSub<std::string>(params(), params.take_id), res->values);
          break;
        case "findRegex"_act:
          set2vector(onto_->data_properties_.findRegex<std::string>(params(), params.take_id), res->values);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set2vector(onto_->data_properties_.findFuzzy(params(), params.take_id, params.threshold), res->values);
          else
            set2vector(onto_->data_properties_.findFuzzy(params(), params.take_id), res->values);
          break;
        case "exist"_act:
          if(onto_->data_properties_.touch(params()))
            res->values.push_back(params());
          break;
        case "getAll"_act:
          res->values = onto_->data_properties_.getAll();
          break;
        case "getComments"_act:
          res->values = onto_->data_properties_.getComments(params());
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector.empty() == false)
        {
          switch(action_hash)
          {
          case "getUp"_act: [[fallthrough]];
          case "getDown"_act: [[fallthrough]];
          case "getDisjoint"_act:
            set_res = onto_->data_properties_.select(set_res, params.selector);
            break;
          case "getDomain"_act:
            set_res = onto_->classes_.select(set_res, params.selector);
            break;
          default:
            break;
          }
        }

        if(res->values.empty())
          set2vector(set_res, res->values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::individualHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Request>& req,
                                      compat::onto_ros::ServiceWrapper<compat::OntologeniusService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      res->code = 0;

      if(onto_->isInit() == false)
        res->code = UNINIT;
      else
      {
        removeUselessSpace(req->action);
        InterfaceParams params;
        params.extractStringParams(req->param);

        reasoner_mutex_.lock();
        reasoners_.runPreReasoners(query_origin_individual, req->action, params());
        reasoner_mutex_.unlock();

        std::unordered_set<std::string> set_res;

        const auto action_hash = hashStr(req->action);

        switch(action_hash)
        {
        case "getSame"_act:
          set_res = onto_->individuals_.getSame(params());
          break;
        case "getDistincts"_act:
          set_res = onto_->individuals_.getDistincts(params());
          break;
        case "getRelationFrom"_act:
          set_res = onto_->individuals_.getRelationFrom(params(), static_cast<int>(params.depth));
          break;
        case "getRelatedFrom"_act:
          set_res = onto_->individuals_.getRelatedFrom(params());
          break;
        case "getRelationOn"_act:
          set_res = onto_->individuals_.getRelationOn(params(), static_cast<int>(params.depth));
          break;
        case "getRelatedOn"_act:
          set_res = onto_->individuals_.getRelatedOn(params());
          break;
        case "getRelationWith"_act:
          set_res = onto_->individuals_.getRelationWith(params());
          break;
        case "getRelatedWith"_act:
          set_res = onto_->individuals_.getRelatedWith(params());
          break;
        case "getUp"_act:
          set_res = onto_->individuals_.getUp(params(), static_cast<int>(params.depth), false);
          break;
        case "getOn"_act:
          set_res = onto_->individuals_.getOn(params());
          break;
        case "getFrom"_act:
          set_res = onto_->individuals_.getFrom(params());
          break;
        case "getWith"_act:
          set_res = onto_->individuals_.getWith(params(), static_cast<int>(params.depth));
          break;
        case "getDomainOf"_act:
          set_res = onto_->individuals_.getDomainOf(params(), static_cast<int>(params.depth));
          break;
        case "getRangeOf"_act:
          set_res = onto_->individuals_.getRangeOf(params(), static_cast<int>(params.depth));
          break;
        case "getName"_act:
        {
          auto tmp = onto_->individuals_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
          break;
        }
        case "getNames"_act:
          res->values = onto_->individuals_.getNames(params(), params.take_id);
          break;
        case "getEveryNames"_act:
          res->values = onto_->individuals_.getEveryNames(params(), params.take_id);
          break;
        case "find"_act:
          set_res = onto_->individuals_.find<std::string>(params(), params.take_id);
          break;
        case "findSub"_act:
          set_res = onto_->individuals_.findSub<std::string>(params(), params.take_id);
          break;
        case "findRegex"_act:
          set_res = onto_->individuals_.findRegex<std::string>(params(), params.take_id);
          break;
        case "findFuzzy"_act:
          if(params.threshold != -1)
            set_res = onto_->individuals_.findFuzzy(params(), params.take_id, params.threshold);
          else
            set_res = onto_->individuals_.findFuzzy(params(), params.take_id);
          break;
        case "getType"_act:
          set_res = onto_->individuals_.getType(params());
          break;
        case "exist"_act:
          if(onto_->individuals_.touch(params()))
            res->values.push_back(params());
          break;
        case "relationExists"_act:
          if(onto_->individuals_.relationExists(params()))
            res->values.push_back(params());
          break;
        case "getAll"_act:
          res->values = onto_->individuals_.getAll();
          break;
        case "isInferred"_act:
          res->values = onto_->individuals_.isInferred(params()) ? std::vector<std::string>{params()} : std::vector<std::string>{""};
          break;
        case "getInferenceExplanation"_act:
          res->values = onto_->individuals_.getInferenceExplanation(params());
          break;
        case "getInferenceRule"_act:
          res->values = {onto_->individuals_.getInferenceRule(params())};
          break;
        case "getComments"_act:
          res->values = onto_->individuals_.getComments(params());
          break;
        default:
          res->code = UNKNOW_ACTION;
        }

        if(params.selector.empty() == false)
        {
          switch(action_hash)
          {
          case "getUp"_act:
            set_res = onto_->classes_.select(set_res, params.selector);
            break;
          case "getRelationFrom"_act: [[fallthrough]];
          case "getRelationOn"_act: [[fallthrough]];
          case "getWith"_act: [[fallthrough]];
          case "getDomainOf"_act: [[fallthrough]];
          case "getRangeOf"_act:
            set_res = onto_->object_properties_.select(set_res, params.selector);
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
            set_res = onto_->individuals_.select(set_res, params.selector);
            break;
          }
        }

        if(res->values.empty())
          set2vector(set_res, res->values);
      }

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

  bool RosInterface::sparqlHandle(compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlService::Request>& req,
                                  compat::onto_ros::ServiceWrapper<compat::OntologeniusSparqlService::Response>& res)
  {
    return [this](auto&& req, auto&& res) {
      const std::pair<std::vector<std::string>, std::vector<std::vector<std::string>>> results = sparql_.runStr(req->query);

      if(results.second.empty() == false)
        res->names = results.first;

      for(const auto& result : results.second)
      {
        compat::OntologeniusSparqlResponse tmp;
        for(const auto& r : result)
          tmp.values.push_back(r);

        res->results.push_back(tmp);
      }

      res->error = sparql_.getError();

      return true;
    }(compat::onto_ros::getServicePointer(req), compat::onto_ros::getServicePointer(res));
  }

} // namespace ontologenius