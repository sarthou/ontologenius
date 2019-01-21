#include "ontoloGenius/core/ontoGraphs/Checkers/ClassChecker.h"
#include <unordered_set>

size_t ClassChecker::check()
{
  std::shared_lock<std::shared_timed_mutex> lock(class_graph_->mutex_);
  graph_size = graph_vect_.size();

  std::unordered_set<std::string> loops_errors = checkLoops();
  /*for(auto err : loops_errors)
  {
    ClassBranch_t* branch = class_graph_->findBranch(err);
    if(branch != nullptr)
      class_graph_->deleteClass(branch);
  }*/
  std::cout << "lopp checked" << std::endl;
  checkDisjoint();

  is_analysed = true;
  printStatus();

  return getErrors();
}

void ClassChecker::checkDisjoint()
{
  for(ClassBranch_t* branch : graph_vect_)
  {
    //std::cout << branch->value() << std::endl;
    std::unordered_set<ClassBranch_t*> up;
    class_graph_->getUpPtr(branch, up);

    std::unordered_set<ClassBranch_t*> disjoint;

    for(ClassBranch_t* it : up)
      class_graph_->getDisjoint(it, disjoint);

    ClassBranch_t* intersection = findIntersection(up, disjoint);
    if(intersection != nullptr)
      print_error("'" + branch->value() + "' can't be a '" + intersection->value() + "' because of disjonction");
  }
}
