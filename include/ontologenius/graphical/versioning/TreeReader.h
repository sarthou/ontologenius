#ifndef ONTOLOGENIUS_TREEREADER_H
#define ONTOLOGENIUS_TREEREADER_H

#include <string>
#include <tinyxml2.h>
#include <vector>

namespace ontologenius {

  class Commit
  {
  public:
    explicit Commit(const std::string& id);
    ~Commit();

    void setOrderId(size_t order);
    void insertData(const std::string& data);
    void insertNext(Commit* next);

    static size_t global_width;
    static size_t global_height;

    std::vector<std::string> datas_;
    std::vector<Commit*> nexts_;
    std::string id_;
    int order_;
  };

  class TreeReader
  {
  public:
    Commit* read(const std::string& file_name);

  private:
    Commit* readNode(tinyxml2::XMLElement* elem);
    Commit* readNode(tinyxml2::XMLElement* elem, Commit* prev);

    std::string getAttribute(tinyxml2::XMLElement* sub_elem, const std::string& attribute);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_TREEREADER_H
