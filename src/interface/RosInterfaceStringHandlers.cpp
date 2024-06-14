#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ontologenius/compat/ros.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/interface/InterfaceParams.h"
#include "ontologenius/interface/RosInterface.h"

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

        if(req->action == "getDown")
          set_res = onto_->class_graph_.getDown(params(), (int)params.depth);
        else if(req->action == "getUp")
          set_res = onto_->class_graph_.getUp(params(), (int)params.depth);
        else if(req->action == "getDisjoint")
          set_res = onto_->class_graph_.getDisjoint(params());
        else if(req->action == "getName")
        {
          auto tmp = onto_->class_graph_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->values = onto_->class_graph_.getNames(params(), params.take_id);
        else if(req->action == "getEveryNames")
          res->values = onto_->class_graph_.getEveryNames(params(), params.take_id);
        else if(req->action == "getRelationFrom")
          set_res = onto_->class_graph_.getRelationFrom(params(), (int)params.depth);
        else if(req->action == "getRelatedFrom")
          set_res = onto_->class_graph_.getRelatedFrom(params());
        else if(req->action == "getRelationOn")
          set_res = onto_->class_graph_.getRelationOn(params(), (int)params.depth);
        else if(req->action == "getRelatedOn")
          set_res = onto_->class_graph_.getRelatedOn(params());
        else if(req->action == "getRelationWith")
          set_res = onto_->class_graph_.getRelationWith(params());
        else if(req->action == "getRelatedWith")
          set_res = onto_->class_graph_.getRelatedWith(params());
        else if(req->action == "getOn")
          set_res = onto_->class_graph_.getOn(params());
        else if(req->action == "getFrom")
          set_res = onto_->class_graph_.getFrom(params());
        else if(req->action == "getWith")
          set_res = onto_->class_graph_.getWith(params(), (int)params.depth);
        else if(req->action == "getDomainOf")
          set_res = onto_->class_graph_.getDomainOf(params(), (int)params.depth);
        else if(req->action == "getRangeOf")
          set_res = onto_->class_graph_.getRangeOf(params(), (int)params.depth);
        else if(req->action == "find")
          set2vector(onto_->class_graph_.find<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findSub")
          set2vector(onto_->class_graph_.findSub<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findRegex")
          set2vector(onto_->class_graph_.findRegex<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set2vector(onto_->class_graph_.findFuzzy(params(), params.take_id, params.threshold), res->values);
          else
            set2vector(onto_->class_graph_.findFuzzy(params(), params.take_id), res->values);
        }
        else if(req->action == "exist")
        {
          if(onto_->class_graph_.touch(params()))
            res->values.push_back(params());
        }
        else if(req->action == "getAll")
          res->values = onto_->class_graph_.getAll();
        else
          res->code = UNKNOW_ACTION;

        if(params.selector.empty() == false)
        {
          if((req->action == "getUp") || (req->action == "getDown") ||
             (req->action == "getDisjoint") || (req->action == "getOn") ||
             (req->action == "getFrom"))
            set_res = onto_->class_graph_.select(set_res, params.selector);
          else if((req->action == "getRelationFrom") || (req->action == "getRelationOn") || (req->action == "getWith") ||
                  (req->action == "getDomainOf") || (req->action == "getRangeOf"))
            set_res = onto_->object_property_graph_.select(set_res, params.selector);
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

        if(req->action == "getDown")
          set_res = onto_->object_property_graph_.getDown(params(), (int)params.depth);
        else if(req->action == "getUp")
          set_res = onto_->object_property_graph_.getUp(params(), (int)params.depth);
        else if(req->action == "getDisjoint")
          set_res = onto_->object_property_graph_.getDisjoint(params());
        else if(req->action == "getInverse")
          set_res = onto_->object_property_graph_.getInverse(params());
        else if(req->action == "getDomain")
          set_res = onto_->object_property_graph_.getDomain(params(), params.depth);
        else if(req->action == "getRange")
          set_res = onto_->object_property_graph_.getRange(params(), params.depth);
        else if(req->action == "getName")
        {
          auto tmp = onto_->object_property_graph_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->values = onto_->object_property_graph_.getNames(params(), params.take_id);
        else if(req->action == "getEveryNames")
          res->values = onto_->object_property_graph_.getEveryNames(params(), params.take_id);
        else if(req->action == "find")
          set2vector(onto_->object_property_graph_.find<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findSub")
          set2vector(onto_->object_property_graph_.findSub<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findRegex")
          set2vector(onto_->object_property_graph_.findRegex<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set2vector(onto_->object_property_graph_.findFuzzy(params(), params.take_id, params.threshold), res->values);
          else
            set2vector(onto_->object_property_graph_.findFuzzy(params(), params.take_id), res->values);
        }
        else if(req->action == "exist")
        {
          if(onto_->object_property_graph_.touch(params()))
            res->values.push_back(params());
        }
        else if(req->action == "getAll")
          res->values = onto_->object_property_graph_.getAll();
        else
          res->code = UNKNOW_ACTION;

        if(params.selector.empty() == false)
        {
          if((req->action == "getUp") || (req->action == "getDown") ||
             (req->action == "getDisjoint") || (req->action == "getInverse"))
            set_res = onto_->object_property_graph_.select(set_res, params.selector);
          else if((req->action == "getDomain") || (req->action == "getRange"))
            set_res = onto_->class_graph_.select(set_res, params.selector);
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

        if(req->action == "getDown")
          set_res = onto_->data_property_graph_.getDown(params(), (int)params.depth);
        else if(req->action == "getUp")
          set_res = onto_->data_property_graph_.getUp(params(), (int)params.depth);
        else if(req->action == "getDisjoint")
          set_res = onto_->data_property_graph_.getDisjoint(params());
        else if(req->action == "getDomain")
          set_res = onto_->data_property_graph_.getDomain(params(), params.depth);
        else if(req->action == "getRange")
          set2vector(onto_->data_property_graph_.getRange(params()), res->values);
        else if(req->action == "getName")
        {
          auto tmp = onto_->data_property_graph_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->values = onto_->data_property_graph_.getNames(params(), params.take_id);
        else if(req->action == "getEveryNames")
          res->values = onto_->data_property_graph_.getEveryNames(params(), params.take_id);
        else if(req->action == "find")
          set2vector(onto_->data_property_graph_.find<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findSub")
          set2vector(onto_->data_property_graph_.findSub<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findRegex")
          set2vector(onto_->data_property_graph_.findRegex<std::string>(params(), params.take_id), res->values);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set2vector(onto_->data_property_graph_.findFuzzy(params(), params.take_id, params.threshold), res->values);
          else
            set2vector(onto_->data_property_graph_.findFuzzy(params(), params.take_id), res->values);
        }
        else if(req->action == "exist")
        {
          if(onto_->data_property_graph_.touch(params()))
            res->values.push_back(params());
        }
        else if(req->action == "getAll")
          res->values = onto_->data_property_graph_.getAll();
        else
          res->code = UNKNOW_ACTION;

        if(params.selector.empty() == false)
        {
          if((req->action == "getUp") || (req->action == "getDown") || (req->action == "getDisjoint"))
            set_res = onto_->data_property_graph_.select(set_res, params.selector);
          else if(req->action == "getDomain")
            set_res = onto_->class_graph_.select(set_res, params.selector);
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

        if(req->action == "getSame")
          set_res = onto_->individual_graph_.getSame(params());
        else if(req->action == "getDistincts")
          set_res = onto_->individual_graph_.getDistincts(params());
        else if(req->action == "getRelationFrom")
          set_res = onto_->individual_graph_.getRelationFrom(params(), (int)params.depth);
        else if(req->action == "getRelatedFrom")
          set_res = onto_->individual_graph_.getRelatedFrom(params());
        else if(req->action == "getRelationOn")
          set_res = onto_->individual_graph_.getRelationOn(params(), (int)params.depth);
        else if(req->action == "getRelatedOn")
          set_res = onto_->individual_graph_.getRelatedOn(params());
        else if(req->action == "getRelationWith")
          set_res = onto_->individual_graph_.getRelationWith(params());
        else if(req->action == "getRelatedWith")
          set_res = onto_->individual_graph_.getRelatedWith(params());
        else if(req->action == "getUp")
          set_res = onto_->individual_graph_.getUp(params(), (int)params.depth);
        else if(req->action == "getOn")
          set_res = onto_->individual_graph_.getOn(params());
        else if(req->action == "getFrom")
          set_res = onto_->individual_graph_.getFrom(params());
        else if(req->action == "getWith")
          set_res = onto_->individual_graph_.getWith(params(), (int)params.depth);
        else if(req->action == "getDomainOf")
          set_res = onto_->individual_graph_.getDomainOf(params(), (int)params.depth);
        else if(req->action == "getRangeOf")
          set_res = onto_->individual_graph_.getRangeOf(params(), (int)params.depth);
        else if(req->action == "getName")
        {
          auto tmp = onto_->individual_graph_.getName(params(), params.take_id);
          if(tmp.empty() == false)
            res->values.push_back(tmp);
        }
        else if(req->action == "getNames")
          res->values = onto_->individual_graph_.getNames(params(), params.take_id);
        else if(req->action == "getEveryNames")
          res->values = onto_->individual_graph_.getEveryNames(params(), params.take_id);
        else if(req->action == "find")
          set_res = onto_->individual_graph_.find<std::string>(params(), params.take_id);
        else if(req->action == "findSub")
          set_res = onto_->individual_graph_.findSub<std::string>(params(), params.take_id);
        else if(req->action == "findRegex")
          set_res = onto_->individual_graph_.findRegex<std::string>(params(), params.take_id);
        else if(req->action == "findFuzzy")
        {
          if(params.threshold != -1)
            set_res = onto_->individual_graph_.findFuzzy(params(), params.take_id, params.threshold);
          else
            set_res = onto_->individual_graph_.findFuzzy(params(), params.take_id);
        }
        else if(req->action == "getType")
          set_res = onto_->individual_graph_.getType(params());
        else if(req->action == "exist")
        {
          if(onto_->individual_graph_.touch(params()))
            res->values.push_back(params());
        }
        else if(req->action == "relationExists")
        {
          if(onto_->individual_graph_.relationExists(params()))
            res->values.push_back(params());
        }
        else if(req->action == "getAll")
          res->values = onto_->individual_graph_.getAll();
        else
          res->code = UNKNOW_ACTION;

        if(params.selector.empty() == false)
        {
          if(req->action == "getUp")
            set_res = onto_->class_graph_.select(set_res, params.selector);
          else if((req->action == "getRelationFrom") || (req->action == "getRelationOn") || (req->action == "getWith") ||
                  (req->action == "getDomainOf") || (req->action == "getRangeOf"))
            set_res = onto_->object_property_graph_.select(set_res, params.selector);
          else if((req->action != "find") || (req->action != "findRegex") || (req->action != "findSub") ||
                  (req->action != "findFuzzy") || (req->action != "getFrom") || (req->action != "getOn"))
            set_res = onto_->individual_graph_.select(set_res, params.selector);
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