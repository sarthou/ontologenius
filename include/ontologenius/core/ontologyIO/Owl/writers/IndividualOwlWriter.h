#ifndef ONTOLOGENIUS_INDIVIDUALWRITER_H
#define ONTOLOGENIUS_INDIVIDUALWRITER_H

#include <string>
#include <vector>

#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"

namespace ontologenius {

  class IndividualGraph;
  class IndividualBranch;

  class IndividualOwlWriter : public NodeOwlWriter
  {
  public:
    IndividualOwlWriter(IndividualGraph* individual_graph, const std::string& ns);
    ~IndividualOwlWriter() = default;

    void write(FILE* file);
    void writeGeneralAxioms(FILE* file);

  private:
    IndividualGraph* individual_graph_;

    void writeIndividual(IndividualBranch* branch);
    void writeType(IndividualBranch* branch);
    void writeObjectProperties(IndividualBranch* branch);
    void writeDataProperties(IndividualBranch* branch);
    void writeSameAs(IndividualBranch* branch);
    void writeDistincts(std::vector<IndividualBranch*>& individuals);
    void getDistincts(IndividualBranch* individual, std::vector<std::string>& distincts_current);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALWRITER_H
