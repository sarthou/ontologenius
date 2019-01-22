#include "ontoloGenius/core/ontoGraphs/Checkers/ClassChecker.h"
#include <unordered_set>

size_t ClassChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
  graph_size = graph_vect_.size();

  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void ClassChecker::checkDisjoint()
{
  for(ClassBranch_t* branch : graph_vect_)
  {
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(branch, up);

    std::unordered_set<ClassBranch_t*> disjoint;

    for(ClassBranch_t* it : up)
    {
      for(ClassBranch_t* dis_i : it->disjoints_)
        class_graph_->getDownPtr(dis_i, disjoint);
    }
    // same as : class_graph_->getDisjoint(it, disjoint);

    ClassBranch_t* intersection = findIntersection(up, disjoint);
    if(intersection != nullptr)
      print_error("'" + branch->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
  }
}
