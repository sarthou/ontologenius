#ifndef ONTOLOGENIUS_FEEDER_H
#define ONTOLOGENIUS_FEEDER_H

#include <string>
#include <vector>

#include "ontologenius/core/feeder/FeedStorage.h"
#include "ontologenius/core/feeder/Versionor.h"

namespace ontologenius {

  class Ontology;

  class Feeder
  {
  public:
    Feeder(Ontology* onto = nullptr, bool versioning = false);

    void store(const std::string& feed, const RosTime_t& stamp) { feed_storage_.add(feed, stamp); }
    bool run();
    void link(Ontology* onto) { onto_ = onto; }
    void setVersioning(bool do_versioning) { do_versioning_ = do_versioning; }
    bool versioning() const { return do_versioning_; }

    std::vector<std::string> getNotifications()
    {
      auto tmp = std::move(notifications_);
      notifications_.clear();
      return tmp;
    }

    std::vector<std::pair<std::string, std::string>> getExplanations()
    {
      auto tmp = std::move(explanations_);
      explanations_.clear();
      return tmp;
    }

    std::vector<std::pair<std::string, RosTime_t>> getValidRelations()
    {
      auto tmp = std::move(valid_relations_);
      valid_relations_.clear();
      return tmp;
    }

    std::vector<std::string> getSynchroMsgs()
    {
      auto tmp = std::move(synchro_msgs_);
      synchro_msgs_.clear();
      return tmp;
    }

    void activateVersionning(bool activated) { versionor_.activate(activated); }
    void exportToXml(const std::string& path) { versionor_.exportToXml(path); }

    std::string getCurrentCommit()
    {
      return versionor_.getCurrentCommit();
    }

    size_t getNbUncommitedData()
    {
      return versionor_.getNbData();
    }

    bool areSameStates(const std::string& commit_from, const std::string& commit_to)
    {
      return versionor_.areSameStates(commit_from, commit_to);
    }

    size_t size() { return feed_storage_.size(); }

  private:
    FeedStorage feed_storage_;
    Versionor versionor_;
    Ontology* onto_;
    bool do_versioning_;

    // Here the notifications are about miss formed queries
    std::vector<std::string> notifications_;
    // Here the explanations are about relations removed because of FOL
    std::vector<std::pair<std::string, std::string>> explanations_;
    // Here the valid relations added to the ontology
    std::vector<std::pair<std::string, RosTime_t>> valid_relations_;
    // Here are the synchro messages to be sent to the client
    std::vector<std::string> synchro_msgs_;

    std::string current_str_feed_;

    bool addFeed(Feed_t& feed);

    bool addDelClass(Action_e& action, std::string& name);
    bool addDelIndiv(Action_e& action, std::string& name);

    bool addInheritage(Feed_t& feed);
    bool addInverseOf(const Feed_t& feed);
    bool addSameAs(const Feed_t& feed);

    bool modifyLangage(Feed_t& feed);
    bool applyProperty(Feed_t& feed);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_FEEDER_H
