#ifndef ONTOLOGENIUS_TREEREADER_H
#define ONTOLOGENIUS_TREEREADER_H

#include <string>
#include <vector>

#include <tinyxml.h>

namespace ontologenius {

class commit_t
{
public:
  commit_t(const std::string& id);
  ~commit_t();

  void insertData(const std::string& data);
  void insertNext(commit_t* next);

  static size_t global_width;
  static size_t global_height;

  std::vector<std::string> datas_;
  std::vector<commit_t*> nexts_;
  int id_;
};

class TreeReader
{
public:
  commit_t* read(const std::string& file_name);
private:
  commit_t* readNode(TiXmlElement* elem);
  commit_t* readNode(TiXmlElement* elem, commit_t* prev);

  std::string getAttribute(TiXmlElement* subElem, const std::string& attribute);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_TREEREADER_H
