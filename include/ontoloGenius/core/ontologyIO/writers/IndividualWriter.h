#ifndef ONTOLOGENIUS_INDIVIDUALWRITER_H
#define ONTOLOGENIUS_INDIVIDUALWRITER_H

#include <string>
#include <vector>

#include "ontoloGenius/core/ontologyIO/writers/NodeWriter.h"

namespace ontologenius {

class IndividualGraph;
class IndividualBranch_t;

class IndividualWriter : public NodeWriter
{
public:
  IndividualWriter(IndividualGraph* individual_graph) {individual_graph_ = individual_graph; };
  ~IndividualWriter() {};

  void write(FILE* file);
  void writeGeneralAxioms(FILE* file);

private:
  IndividualGraph* individual_graph_;

  void writeIndividual(IndividualBranch_t* branch);
  void writeType(IndividualBranch_t* branch);
  void writeObjectProperties(IndividualBranch_t* branch);
  void writeObjectPropertiesDeduced(IndividualBranch_t* branch);
  void writeDataProperties(IndividualBranch_t* branch);
  void writeDataPropertiesDeduced(IndividualBranch_t* branch);
  void writeSameAs(IndividualBranch_t* branch);
  void writeDistincts(std::vector<IndividualBranch_t*>& individuals);
  void getDistincts(IndividualBranch_t* individual, std::vector<std::string>& distincts_current);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_INDIVIDUALWRITER_H
