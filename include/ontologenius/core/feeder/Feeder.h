#ifndef ONTOLOGENIUS_FEEDER_H
#define ONTOLOGENIUS_FEEDER_H

#include "ontologenius/core/feeder/FeedStorage.h"
#include "ontologenius/core/feeder/Versionor.h"

namespace ontologenius {

class Ontology;

class Feeder
{
public:
  Feeder(Ontology* onto = nullptr);

  void store(std::string feed) { feed_storage_.add(feed); }
  bool run();
  void link(Ontology* onto) {onto_ = onto; }

  std::vector<std::string> getNotifications()
  {
    std::vector<std::string> tmp = notifications_;
    notifications_.clear();
    return tmp;
  }

  void activateVersionning(bool activated) { versionor_.activate(activated); }
  void exportToXml(const std::string& path) { versionor_.exportToXml(path); }

  size_t size() { return feed_storage_.size(); }

private:
  FeedStorage feed_storage_;
  Versionor versionor_;
  Ontology* onto_;

  std::vector<std::string> notifications_;
  std::string current_str_feed_;

  void addDelClass(action_t& action, std::string& name);
  void addDelIndiv(action_t& action, std::string& name);

  void addInheritage(feed_t& feed);
  void modifyDataPropertyInheritance(feed_t& feed);
  void modifyDataPropertyInheritanceInvert(feed_t& feed);
  void modifyObjectPropertyInheritance(feed_t& feed);
  void modifyObjectPropertyInheritanceInvert(feed_t& feed);
  void classIndividualIsA(feed_t& feed);

  void classIndividualLangage(feed_t& feed);
  void applyProperty(feed_t& feed);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDER_H
