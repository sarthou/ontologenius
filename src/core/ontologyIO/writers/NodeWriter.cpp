#include "ontoloGenius/core/ontologyIO/writers/NodeWriter.h"

#include <map>
#include <vector>

#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

namespace ontologenius {

void NodeWriter::writeDictionary(ValuedNodeData* node)
{
  for(auto& it : node->dictionary_)
  {
    for(size_t i = 0; i < it.second.size(); i++)
    {
      std::string tmp = "        <rdfs:label xml:lang=\"" +
                        it.first +
                        "\">" +
                        it.second[i] +
                        + "</rdfs:label>\n\r";
      writeString(tmp);
    }
  }
}

void NodeWriter::writeMutedDictionary(ValuedNodeData* node)
{
  for(auto& it : node->muted_dictionary_)
  {
    for(size_t i = 0; i < it.second.size(); i++)
    {
      std::string tmp = "        <onto:label xml:lang=\"" +
                        it.first +
                        "\">" +
                        it.second[i] +
                        + "</onto:label>\n\r";
      writeString(tmp);
    }
  }
}

void NodeWriter::writeString(std::string text)
{
  if(file_ != NULL)
    fwrite(text.c_str(), sizeof(char), text.size(), file_);
}

} // namespace ontologenius
