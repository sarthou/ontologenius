#include "ontoloGenius/core/ontologyIO/writers/NodeWriter.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

#include <map>
#include <vector>

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

void NodeWriter::writeString(std::string text)
{
  if(file_ != NULL)
    fwrite(text.c_str(), sizeof(char), text.size(), file_);
}

} // namespace ontologenius
