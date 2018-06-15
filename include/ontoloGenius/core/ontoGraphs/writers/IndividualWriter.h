#ifndef INDIVIDUALWRITER_H
#define INDIVIDUALWRITER_H

#include "ontoloGenius/core/ontoGraphs/writers/NodeWriter.h"

#include <string>

class IndividualGraph;
class IndividualBranch_t;

class IndividualWriter : public NodeWriter
{
public:
  IndividualWriter(IndividualGraph* individual_graph) {individual_graph_ = individual_graph; };
  ~IndividualWriter() {};

  void write(FILE* file);

private:
  IndividualGraph* individual_graph_;

  void writeIndividual(IndividualBranch_t* branch);
  void writeType(IndividualBranch_t* branch);
};

#endif
