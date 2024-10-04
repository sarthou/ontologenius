#include "ontologenius/core/ontologyIO/Owl/writers/RuleOwlWriter.h"

#include <shared_mutex>

#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"

namespace ontologenius {

  RuleOwlWriter::RuleOwlWriter(RuleGraph* rule_graph, const std::string& ns) : rule_graph_(rule_graph)
  {
    ns_ = ns;
  }

  void RuleOwlWriter::write(FILE* file)
  {
    file_ = file;

    const std::shared_lock<std::shared_timed_mutex> lock(rule_graph_->mutex_);

    const std::vector<RuleBranch*> rules = rule_graph_->get();

    for(auto* rule : rules)
      writeRule(rule);

    file_ = nullptr;
  }

  void RuleOwlWriter::writeRule(RuleBranch* branch)
  {
  }


} // namespace ontologenius