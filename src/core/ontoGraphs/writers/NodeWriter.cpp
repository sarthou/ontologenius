#include "ontoloGenius/core/ontoGraphs/writers/NodeWriter.h"

#include "ontoloGenius/core/ontoGraphs/Branchs/ValuedNode.h"

#include <map>
#include <vector>

void NodeWriter::writeDictionary(ValuedNodeData* node)
{
  std::map<std::string, std::vector<std::string>>::iterator it;
  for(it = node->dictionary_.begin(); it != node->dictionary_.end(); ++it)
  {
    for(size_t i = 0; i < it->second.size(); i++)
    {
      std::string tmp = "        <rdfs:label xml:lang=\"" +
                        it->first +
                        "\">" +
                        it->second[i] +
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
