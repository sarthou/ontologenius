#ifndef ONTOLOGENIUS_NODEWRITER_H
#define ONTOLOGENIUS_NODEWRITER_H

#include <string>

namespace ontologenius {

class ValuedNodeData;

class NodeWriter
{
public:
  NodeWriter() {file_ = nullptr; };
  ~NodeWriter() {};

  FILE* file_;

  void writeDictionary(ValuedNodeData* node);

  void writeString(std::string text);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_NODEWRITER_H
