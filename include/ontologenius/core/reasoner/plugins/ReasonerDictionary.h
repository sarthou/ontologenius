#ifndef ONTOLOGENIUS_REASONERDICTIONARY_H
#define ONTOLOGENIUS_REASONERDICTIONARY_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

  class ReasonerDictionary : public ReasonerInterface
  {
  public:
    ReasonerDictionary() : use_id_(false) {}
    ~ReasonerDictionary() override = default;

    void setParameter(const std::string& name, const std::string& value) override;

    void postReason() override;

    bool implementPostReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return true; }

  private:
    bool use_id_;

    void updateDictionary(ValuedNode* node);

    void setId(ValuedNode* node);
    void split(ValuedNode* node);
    void createLowerCase(ValuedNode* node);
    void replaceQuote(ValuedNode* node);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERDICTIONARY_H
