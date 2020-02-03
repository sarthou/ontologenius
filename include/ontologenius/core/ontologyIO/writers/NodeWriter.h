#ifndef ONTOLOGENIUS_NODEWRITER_H
#define ONTOLOGENIUS_NODEWRITER_H

#include <string>

#include "ontologenius/core/ontoGraphs/Branchs/Elements.h"
#include "ontologenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

class NodeWriter
{
public:
  NodeWriter() {file_ = nullptr; };
  ~NodeWriter() {};

  FILE* file_;

  void writeDictionary(ValuedNode* node);
  void writeMutedDictionary(ValuedNode* node);

  void writeString(std::string text);

  template<typename T>
  std::string getProba(Single_t<T>& element)
  {
    return (element < 1.0) ? " onto:probability=\"" + std::to_string(element.probability) + "\"" : "";
  }

  template<typename T, typename U>
  std::string getProba(Pair_t<T, U>& element)
  {
    return (element < 1.0) ? " onto:probability=\"" + std::to_string(element.probability) + "\"" : "";
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_NODEWRITER_H
