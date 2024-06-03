#ifndef ONTOLOGENIUS_BRANCHCONTAINERBASE_H
#define ONTOLOGENIUS_BRANCHCONTAINERBASE_H

#include <string>
#include <vector>

namespace ontologenius {

  class ValuedNode;
  class LiteralNode;
  template<typename T>
  class Branch;

  template<typename B>
  class BranchContainerBase
  {
    static_assert(std::is_base_of<ValuedNode, B>::value || std::is_base_of<LiteralNode, B>::value, "B must be derived from ValuedNode or LiteralNode");

  public:
    BranchContainerBase() = default;
    virtual ~BranchContainerBase() = default;

    virtual B* find(const std::string& word) = 0;
    virtual std::vector<B*> find(bool (*comp)(B*, const std::string&, const std::string&, bool), const std::string& word, const std::string& lang, bool use_default) = 0;
    virtual void load(std::vector<B*>& vect) = 0;
    virtual void insert(B* branch) = 0;
    virtual void erase(B* branch) = 0;

  private:
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_BRANCHCONTAINERBASE_H
