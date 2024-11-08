#ifndef ONTOLOGENIUS_IDMANAGER_H
#define ONTOLOGENIUS_IDMANAGER_H

#include <set>

namespace ontologenius {

  template<typename TInteger>
  class IdManager
  {
    static_assert(std::is_integral<TInteger>::value, "TInteger must be an integral type");

  public:
    IdManager();

    TInteger getNewId();
    bool removeId(TInteger id);

    std::set<TInteger> getIds() { return ids_; }

  private:
    TInteger current_id_;
    std::set<TInteger> ids_;
  };

  template<typename TInteger>
  IdManager<TInteger>::IdManager() : current_id_(0)
  {}

  template<typename TInteger>
  TInteger IdManager<TInteger>::getNewId()
  {
    while(ids_.find(current_id_) != ids_.end())
      current_id_++;

    ids_.insert(current_id_);

    TInteger res = current_id_;
    current_id_++;

    return (res);
  }

  template<typename TInteger>
  bool IdManager<TInteger>::removeId(TInteger id)
  {
    auto it = ids_.find(id);
    if(it == ids_.end())
      return false;
    else
    {
      ids_.erase(it);
      if(id < current_id_)
        current_id_ = id;

      return true;
    }
  }

} // namespace ontologenius

#endif // ONTOLOGENIUS_IDMANAGER_H
