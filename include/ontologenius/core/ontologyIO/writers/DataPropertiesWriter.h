#ifndef ONTOLOGENIUS_DATAPROPERTIESWRITER_H
#define ONTOLOGENIUS_DATAPROPERTIESWRITER_H

#include <string>

#include "ontologenius/core/ontologyIO/writers/PropertiesWriter.h"

namespace ontologenius {

class DataPropertyGraph;
class DataPropertyBranch_t;

class DataPropertiesWriter : public PropertiesWriter<DataPropertyBranch_t>
{
public:
  DataPropertiesWriter(DataPropertyGraph* property_graph, const std::string& ns);
  ~DataPropertiesWriter() {};

  void write(FILE* file);

private:
  DataPropertyGraph* property_graph_;

  void writeProperty(DataPropertyBranch_t* branch);
  void writeSubPropertyOf(DataPropertyBranch_t* branch);
  void writeRange(DataPropertyBranch_t* branch);
  void writeDomain(DataPropertyBranch_t* branch);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTIESWRITER_H
