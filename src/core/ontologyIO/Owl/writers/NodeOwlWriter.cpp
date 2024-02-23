#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"

#include <map>
#include <vector>

namespace ontologenius {

void NodeOwlWriter::writeDictionary(ValuedNode* node)
{
  for(auto& it : node->steady_dictionary_.spoken_)
  {
    for(size_t i = 0; i < it.second.size(); i++)
    {
      std::string tmp = "<rdfs:label xml:lang=\"" +
                        it.first +
                        "\">" +
                        it.second[i] +
                        + "</rdfs:label>\n";
      writeString(tmp, 2);
    }
  }
}

void NodeOwlWriter::writeMutedDictionary(ValuedNode* node)
{
  for(auto& it : node->steady_dictionary_.muted_)
  {
    for(size_t i = 0; i < it.second.size(); i++)
    {
      std::string tmp = "<onto:label xml:lang=\"" +
                        it.first +
                        "\">" +
                        it.second[i] +
                        + "</onto:label>\n";
      writeString(tmp, 2);
    }
  }
}

void NodeOwlWriter::writeString(const std::string& text, size_t level)
{
  if(file_ != nullptr)
  {
    std::string str(level*4, ' ');
    str += text;
    fwrite(str.c_str(), sizeof(char), str.size(), file_);
  }
}

} // namespace ontologenius
