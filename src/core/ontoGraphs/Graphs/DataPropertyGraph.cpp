#include "ontologenius/core/ontoGraphs/Graphs/DataPropertyGraph.h"

#include <iostream>

#include "ontologenius/core/ontoGraphs/Graphs/ClassGraph.h"

namespace ontologenius {

DataPropertyGraph::DataPropertyGraph(ClassGraph* class_graph)
{
  class_graph_ = class_graph;
}

DataPropertyGraph::DataPropertyGraph(const DataPropertyGraph& other, ClassGraph* class_graph)
{
  class_graph_ = class_graph;

  language_ = other.language_;

  for(const auto& root : other.roots_)
  {
    auto prop_branch = new DataPropertyBranch_t(root.first);
    roots_[root.first] = prop_branch;
    all_branchs_.push_back(prop_branch);
  }

  for(const auto& branch : other.branchs_)
  {
    auto prop_branch = new DataPropertyBranch_t(branch.first);
    branchs_[branch.first] = prop_branch;
    all_branchs_.push_back(prop_branch);
  }

  this->container_.load(all_branchs_);
}

DataPropertyBranch_t* DataPropertyGraph::newDefaultBranch(const std::string& name)
{
  auto branch = new DataPropertyBranch_t(name);
  roots_[name] = branch;
  all_branchs_.push_back(branch);
  container_.insert(branch);
  return branch;
}

DataPropertyBranch_t* DataPropertyGraph::add(const std::string& value, DataPropertyVectors_t& property_vectors, bool direct_load)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);
  /**********************
  ** Mothers
  **********************/
  DataPropertyBranch_t* me = nullptr;
  //am I a created mother ?
  amIA(&me, tmp_mothers_, value);

  //am I a created branch ?
  amIA(&me, branchs_, value);

  //am I a created root ?
  amIA(&me, roots_, value);

  //am I created ?
  if(me == nullptr)
  {
    me = new DataPropertyBranch_t(value);
    if(direct_load)
    {
      all_branchs_.push_back(me);
      container_.insert(me);
    }
  }

  //am I a root ?
  if(me->mothers_.size() + property_vectors.mothers_.size() == 0)
    roots_[me->value()] = me;
  else
  {
    //for all my mothers
    for(auto& mother : property_vectors.mothers_)
    {
      DataPropertyBranch_t* mother_branch = nullptr;
      getInMap(&mother_branch, mother.elem, roots_);
      getInMap(&mother_branch, mother.elem, branchs_);
      getInMap(&mother_branch, mother.elem, tmp_mothers_);
      if(mother_branch == nullptr)
      {
        mother_branch = new DataPropertyBranch_t(mother.elem);
        tmp_mothers_[mother_branch->value()] = mother_branch;
      }

      conditionalPushBack(mother_branch->childs_, DataPropertyElement_t(me, mother.probability, true));
      conditionalPushBack(me->mothers_, DataPropertyElement_t(mother_branch, mother.probability));
    }

    //but i am also a branch
    branchs_[me->value()] = me;
  }

  /**********************
  ** Disjoints
  **********************/
  //for all my disjoints
  for(auto& disjoint : property_vectors.disjoints_)
  {
    DataPropertyBranch_t* disjoint_branch = nullptr;
    getInMap(&disjoint_branch, disjoint.elem, roots_);
    getInMap(&disjoint_branch, disjoint.elem, branchs_);
    getInMap(&disjoint_branch, disjoint.elem, tmp_mothers_);

    //I create my disjoint
    if(disjoint_branch == nullptr)
    {
      disjoint_branch = new DataPropertyBranch_t(disjoint.elem);
      tmp_mothers_[disjoint_branch->value()] = disjoint_branch; //I put my disjoint as tmp_mother
    }
    conditionalPushBack(me->disjoints_, DataPropertyElement_t(disjoint_branch, disjoint.probability));
    conditionalPushBack(disjoint_branch->disjoints_, DataPropertyElement_t(me, disjoint.probability, true));
  }

  /**********************
  ** Domains
  **********************/
  //for all my domains
  for(auto& domain : property_vectors.domains_)
  {
    ClassBranch_t* domain_branch = nullptr;
    getInMap(&domain_branch, domain.elem, class_graph_->roots_);
    getInMap(&domain_branch, domain.elem, class_graph_->branchs_);
    getInMap(&domain_branch, domain.elem, class_graph_->tmp_mothers_);

    //I create my domain
    if(domain_branch == nullptr)
    {
      ObjectVectors_t empty_vectors;
      class_graph_->add(domain.elem, empty_vectors);
      getInMap(&domain_branch, domain.elem, class_graph_->roots_);
    }
    conditionalPushBack(me->domains_, ClassElement_t(domain_branch, domain.probability));
  }

  /**********************
  ** Ranges
  **********************/
  //for all my ranges
  for(const auto& range : property_vectors.ranges_)
  {
    LiteralNode* literal = createLiteralUnsafe(range + "#");
    conditionalPushBack(me->ranges_, literal);
  }

  /**********************
  ** Language and properties
  **********************/
  me->properties_.apply(property_vectors.properties_);
  me->annotation_usage_ = me->annotation_usage_ || property_vectors.annotation_usage_;
  me->setSteady_dictionary(property_vectors.dictionary_);
  me->setSteady_muted_dictionary(property_vectors.muted_dictionary_);

  mitigate(me);
  return me;
}

void DataPropertyGraph::add(std::vector<std::string>& disjoints)
{
  std::lock_guard<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  for(size_t disjoints_i = 0; disjoints_i < disjoints.size(); disjoints_i++)
  {
    //I need to find myself
    DataPropertyBranch_t* me = nullptr;
    //Am I a root ?
    amIA(&me, roots_, disjoints[disjoints_i], false);

    //Am I a branch ?
    amIA(&me, branchs_, disjoints[disjoints_i], false);

    //Am I a tmp_mother ?
    amIA(&me, tmp_mothers_, disjoints[disjoints_i], false);

    // I don't exist ? so I will be a tmp_mother
    if(me == nullptr)
    {
      me = new DataPropertyBranch_t(disjoints[disjoints_i]);
      tmp_mothers_[me->value()] = me;
    }

    //for all my disjoints ...
    for(size_t disjoints_j = 0; disjoints_j < disjoints.size(); disjoints_j++)
    {
      //... excepted me
      if(disjoints_i != disjoints_j)
      {
        DataPropertyBranch_t* disjoint_branch = nullptr;
        getInMap(&disjoint_branch, disjoints[disjoints_j], roots_);
        getInMap(&disjoint_branch, disjoints[disjoints_j], branchs_);
        getInMap(&disjoint_branch, disjoints[disjoints_j], tmp_mothers_);

        //I create my disjoint
        if(disjoint_branch == nullptr)
        {
          disjoint_branch = new DataPropertyBranch_t(disjoints[disjoints_j]);
          tmp_mothers_[disjoint_branch->value()] = disjoint_branch; //I put my disjoint as tmp_mother
        }
        conditionalPushBack(me->disjoints_, DataPropertyElement_t(disjoint_branch));
        conditionalPushBack(disjoint_branch->disjoints_, DataPropertyElement_t(me, 1.0, true));
      }
    }
  }
}

bool DataPropertyGraph::addAnnotation(const std::string& value, DataPropertyVectors_t& property_vectors)
{
  /**********************
  ** Mothers
  **********************/
  DataPropertyBranch_t* me = nullptr;
  //am I a created mother ?
  amIA(&me, tmp_mothers_, value, false);

  //am I a created branch ?
  amIA(&me, branchs_, value, false);

  //am I a created root ?
  amIA(&me, roots_, value, false);

  //am I created ?
  if(me == nullptr)
  {
    DataPropertyBranch_t* mother_branch = nullptr;
    for(auto& mother : property_vectors.mothers_)
    {
      getInMap(&mother_branch, mother.elem, roots_);
      getInMap(&mother_branch, mother.elem, branchs_);
      getInMap(&mother_branch, mother.elem, tmp_mothers_);
      if(mother_branch != nullptr)
        break;
    }

    // I do not exist but one of my mother do so I should exist
    if(mother_branch != nullptr)
    {
      add(value, property_vectors);
      return true;
    }
    else
    {
      ClassBranch_t* range_branch = nullptr;
      for(auto& range : property_vectors.ranges_)
      {
        getInMap(&range_branch, range, class_graph_->roots_);
        getInMap(&range_branch, range, class_graph_->branchs_);
        getInMap(&range_branch, range, class_graph_->tmp_mothers_);
        if(range_branch != nullptr)
          break;
      }

      // My ranges are not classes so there are data and I should exists
      if((range_branch == nullptr) && (property_vectors.ranges_.size()))
      {
        add(value, property_vectors);
        return true;
      }
      else
        return false;
    }
  }
  else
  {
    add(value, property_vectors);
    return true;
  }
}

LiteralNode* DataPropertyGraph::createLiteral(const std::string& value)
{
  LiteralNode* literal = nullptr;
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    literal = literal_container_.find(value);
  }

  if(literal == nullptr)
  {
    std::lock_guard<std::shared_timed_mutex> lock(mutex_);
    literal = new LiteralNode(value);
    literal_container_.insert(literal);
  }
  return literal;
}

LiteralNode* DataPropertyGraph::createLiteralUnsafe(const std::string& value)
{
  LiteralNode* literal = nullptr;
  literal = literal_container_.find(value);

  if(literal == nullptr)
  {
    literal = new LiteralNode(value);
    literal_container_.insert(literal);
  }
  return literal;
}

std::unordered_set<std::string> DataPropertyGraph::getDisjoint(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(auto& disjoint : branch->disjoints_)
      getDown(disjoint.elem, res);

  return res;
}

std::unordered_set<index_t> DataPropertyGraph::getDisjoint(index_t value)
{
  std::unordered_set<index_t> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(ValuedNode::table_.get(value));
  if(branch != nullptr)
    for(auto& disjoint : branch->disjoints_)
      getDown(disjoint.elem, res);

  return res;
}

std::unordered_set<std::string> DataPropertyGraph::getDomain(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(auto& domain : branch->domains_)
      class_graph_->getDown(domain.elem, res);

  return res;
}

std::unordered_set<index_t> DataPropertyGraph::getDomain(index_t value)
{
  std::unordered_set<index_t> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(ValuedNode::table_.get(value));
  if(branch != nullptr)
    for(auto& domain : branch->domains_)
      class_graph_->getDown(domain.elem, res);

  return res;
}

void DataPropertyGraph::getDomainPtr(DataPropertyBranch_t* branch, std::unordered_set<ClassBranch_t*>& res, size_t depth)
{
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  if(branch != nullptr)
    for(auto& domain : branch->domains_)
      class_graph_->getDownPtr(domain.elem, res, depth);
}

std::unordered_set<std::string> DataPropertyGraph::getRange(const std::string& value)
{
  std::unordered_set<std::string> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(value);
  if(branch != nullptr)
    for(auto& range : branch->ranges_)
      res.insert(range->type_);

  return res;
}

std::unordered_set<index_t> DataPropertyGraph::getRange(index_t value)
{
  std::unordered_set<index_t> res;
  std::shared_lock<std::shared_timed_mutex> lock(Graph<DataPropertyBranch_t>::mutex_);

  DataPropertyBranch_t* branch = container_.find(ValuedNode::table_.get(value));
  if(branch != nullptr)
    for(auto& range : branch->ranges_)
      res.insert(range->get());

  return res;
}

bool DataPropertyGraph::add(DataPropertyBranch_t* prop, const std::string& relation, const std::string& data)
{
  if(relation != "")
  {
    if(relation[0] == '@')
    {
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      prop->setSteady_dictionary(relation.substr(1), data);
      prop->updated_ = true;
    }
    else if((relation == "+") || (relation == "isA"))
    {
      DataPropertyBranch_t* tmp = create(data);
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      conditionalPushBack(prop->mothers_, DataPropertyElement_t(tmp));
      conditionalPushBack(tmp->childs_, DataPropertyElement_t(prop));
      prop->updated_ = true;
      tmp->updated_ = true;
    }
    else
      return false;
  }
  else
    return false;
  return true;
}

bool DataPropertyGraph::addInvert(DataPropertyBranch_t* prop, const std::string& relation, const std::string& data)
{
  if(relation != "")
  {
    if((relation == "+") || (relation == "isA"))
    {
      DataPropertyBranch_t* tmp = create(data);
      std::lock_guard<std::shared_timed_mutex> lock(mutex_);
      conditionalPushBack(tmp->mothers_, DataPropertyElement_t(prop));
      conditionalPushBack(prop->childs_, DataPropertyElement_t(tmp));
      prop->updated_ = true;
      tmp->updated_ = true;
    }
    else
      return false;
  }
  else
    return false;
  return true;
}

bool DataPropertyGraph::remove(DataPropertyBranch_t* prop, const std::string& relation, const std::string& data)
{
  (void)prop;
  (void)relation;
  (void)data;
  return false;
}

void DataPropertyGraph::deepCopy(const DataPropertyGraph& other)
{
  for(const auto& root : other.roots_)
    cpyBranch(root.second, roots_[root.first]);

  for(const auto& branch : other.branchs_)
    cpyBranch(branch.second, branchs_[branch.first]);
}

void DataPropertyGraph::cpyBranch(DataPropertyBranch_t* old_branch, DataPropertyBranch_t* new_branch)
{
  new_branch->nb_updates_ = old_branch->nb_updates_;
  new_branch->updated_ = old_branch->updated_;
  new_branch->flags_ = old_branch->flags_;

  new_branch->dictionary_ = old_branch->dictionary_;
  new_branch->steady_dictionary_ = old_branch->steady_dictionary_;

  for(const auto& child : old_branch->childs_)
    new_branch->childs_.emplace_back(child, container_.find(child.elem->value()));

  for(const auto& mother : old_branch->mothers_)
    new_branch->mothers_.emplace_back(mother, container_.find(mother.elem->value()));

  new_branch->ranges_ = old_branch->ranges_;

  for(const auto& domain : old_branch->domains_)
    new_branch->domains_.emplace_back(domain, class_graph_->container_.find(domain.elem->value()));

  new_branch->properties_ = old_branch->properties_;

  for(const auto& disjoint : old_branch->disjoints_)
    new_branch->disjoints_.emplace_back(disjoint, container_.find(disjoint.elem->value()));
}

} // namespace ontologenius
