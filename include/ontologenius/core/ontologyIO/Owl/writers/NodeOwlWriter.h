#ifndef ONTOLOGENIUS_NODEOWLWRITER_H
#define ONTOLOGENIUS_NODEOWLWRITER_H

#include <string>

#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

  class NodeOwlWriter
  {
  public:
    NodeOwlWriter() : file_(nullptr) {}
    ~NodeOwlWriter() = default;

    FILE* file_;
    std::string ns_;

    void writeDictionary(ValuedNode* node) const;
    void writeMutedDictionary(ValuedNode* node) const;

    void writeString(const std::string& text, size_t level = 0) const;

    template<typename T>
    std::string getProba(SingleElement<T>& element)
    {
      return (element < 1.0) ? " onto:probability=\"" + std::to_string(element.probability) + "\"" : "";
    }

    template<typename T, typename U>
    std::string getProba(PairElement<T, U>& element)
    {
      return (element < 1.0) ? " onto:probability=\"" + std::to_string(element.probability) + "\"" : "";
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_NODEOWLWRITER_H
