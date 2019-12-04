#ifndef ONTOLOGENIUS_CONFIGREADER_H
#define ONTOLOGENIUS_CONFIGREADER_H

#include <string>
#include <vector>
#include <map>
#include <experimental/optional>

namespace ontologenius
{

class ConfigElement
{
public:
  std::experimental::optional<std::vector<std::string>> data;
  std::experimental::optional<std::map<std::string, ConfigElement>> subelem;

  ConfigElement operator[](std::string name)
  {
    if(subelem)
      return subelem.value()[name];
    else
      return ConfigElement();
  }

  std::vector<std::string> value()
  {
    if(data)
      return data.value();
    else
      return {};
  }
};

class ConfigReader
{
public:
  std::map<std::string, ConfigElement> config_;

  bool read(const std::string& path);

  void display();

private:
  void display(std::map<std::string, ConfigElement>& config, size_t nb = 0);
  void displayTab(size_t nb);
  void removeComment(std::string& line);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CONFIGREADER_H
