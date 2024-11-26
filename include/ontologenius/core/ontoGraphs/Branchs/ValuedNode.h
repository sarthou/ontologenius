#ifndef ONTOLOGENIUS_VALUEDNODE_H
#define ONTOLOGENIUS_VALUEDNODE_H

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/WordTable.h"

namespace ontologenius {

  class UpdatableNode
  {
  public:
    unsigned int nb_updates_;

    std::map<std::string, std::vector<std::string>> flags_;

    UpdatableNode() : nb_updates_(0), updated_(true)
    {}

    bool isUpdated() const { return updated_; }
    void setUpdated(bool value) { updated_ = value; }

  protected:
    bool updated_;
  };

  class Dictionary
  {
  public:
    std::map<std::string, std::vector<std::string>> spoken_;
    std::map<std::string, std::vector<std::string>> muted_;
  };

  class ValuedNode : public UpdatableNode
  {
  public:
    explicit ValuedNode(const std::string& value, bool hidden = false) : index_(table.add(value)),
                                                                         hidden_(hidden)
    {}

    const index_t& get() const { return index_; }
    const std::string& value() const { return table[index_]; }

    bool isHidden() const { return hidden_; }

    static WordTable table;

    Dictionary dictionary_;
    Dictionary steady_dictionary_;

    template<typename C>
    inline void conditionalPushBack(std::vector<C>& vect, const C& data)
    {
      if(std::find(vect.begin(), vect.end(), data) == vect.end())
        vect.push_back(data);
    }

    void setSteadyDictionary(const std::string& lang, const std::string& word);
    void setSteadyMutedDictionary(const std::string& lang, const std::string& word);
    void setSteadyDictionary(const std::map<std::string, std::vector<std::string>>& dictionary);
    void setSteadyMutedDictionary(const std::map<std::string, std::vector<std::string>>& dictionary);

  private:
    index_t index_;
    bool hidden_;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_VALUEDNODE_H
