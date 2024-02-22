#ifndef ONTOLOGENIUS_CONFIGREADER_H
#define ONTOLOGENIUS_CONFIGREADER_H

#include <algorithm>
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
    {
      if(subelem.value().find(name) != subelem.value().end())
        return subelem.value()[name];
      else
        return ConfigElement();
    }
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

  ConfigElement operator[](const std::string& name)
  {
    if(config_.find(name) != config_.end())
      return config_[name];
    else
      return ConfigElement();
  }

  std::vector<std::string> getKeys()
  {
    std::vector<std::string> res;
    std::transform(config_.cbegin(), config_.cend(),
                    std::back_inserter(res),
                    [](const std::pair<std::string, ConfigElement>& elem){ return elem.first;});
    return res;
  }

private:
  std::map<std::string, ConfigElement> read(const std::vector<std::string>& lines, size_t& current_line);
  ConfigElement readList(const std::vector<std::string>& lines, size_t& current_line);

  void display(const std::map<std::string, ConfigElement>& config, size_t nb = 0);
  void displayElement(const std::pair<std::string, ConfigElement>& it, size_t nb);
  void displayTab(size_t nb);
  void removeComment(std::string& line);
  size_t countSpaces(const std::string& line);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_CONFIGREADER_H
