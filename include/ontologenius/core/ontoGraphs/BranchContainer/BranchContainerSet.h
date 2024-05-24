#ifndef ONTOLOGENIUS_BRANCHCONTAINERSET_H
#define ONTOLOGENIUS_BRANCHCONTAINERSET_H

#include <map>
#include <set>

#include "ontologenius/core/ontoGraphs/BranchContainer/BranchContainerBase.h"

namespace ontologenius {

  template<typename B>
  struct NodeSetCompare
  {
    using is_transparent = std::true_type;

    bool operator()(const B* a, const std::string& name) const
    {
      return (a->value() < name);
    }

    bool operator()(const std::string& name, const B* a) const
    {
      return (name < a->value());
    }

    bool operator()(const B* a, const B* b) const
    {
      return (a->value() < b->value());
    }
  };

  template<typename B>
  class BranchContainerSet : public BranchContainerBase<B>
  {
  public:
    BranchContainerSet() {}
    BranchContainerSet(const BranchContainerSet& base);
    virtual ~BranchContainerSet() {} // B* is destructed by ontograph

    virtual B* find(const std::string& word) override;
    virtual std::vector<B*> find(bool (*comp)(B*, const std::string&, const std::string&, bool), const std::string& word, const std::string& lang, bool use_default) override;
    virtual void load(std::vector<B*>& vect) override;
    virtual void insert(B* branch) override;
    virtual void erase(B* branch) override;

  private:
    std::set<B*, NodeSetCompare<B>> nodes_;
  };

  template<typename B>
  BranchContainerSet<B>::BranchContainerSet(const BranchContainerSet& base)
  {
    for(auto& it : base.nodes_)
    {
      B* tmp = new B(it);
      nodes_.insert(tmp);
    }
  }

  template<typename B>
  B* BranchContainerSet<B>::find(const std::string& word)
  {
    auto it = nodes_.find(word);
    if(it == nodes_.end())
      return nullptr;
    else
      return *it;
  }

  template<typename B>
  std::vector<B*> BranchContainerSet<B>::find(bool (*comp)(B*, const std::string&, const std::string&, bool), const std::string& word, const std::string& lang, bool use_default)
  {
    std::vector<B*> res;

    for(auto& it : nodes_)
    {
      try
      {
        if(comp(it, word, lang, use_default))
          res.push_back(it);
      }
      catch(...)
      {
        return res;
      }
    }

    return res;
  }

  template<typename B>
  void BranchContainerSet<B>::load(std::vector<B*>& vect)
  {
    for(auto& elem : vect)
      nodes_.insert(elem);
  }

  template<typename B>
  void BranchContainerSet<B>::insert(B* branch)
  {
    nodes_.insert(branch);
  }

  template<typename B>
  void BranchContainerSet<B>::erase(B* branch)
  {
    nodes_.erase(branch);
  }

} // namespace ontologenius

#endif // ONTOLOGENIUS_BRANCHCONTAINERSET_H
