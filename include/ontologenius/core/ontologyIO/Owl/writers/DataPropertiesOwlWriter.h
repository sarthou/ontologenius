#ifndef ONTOLOGENIUS_DATAPROPERTIESOWLWRITER_H
#define ONTOLOGENIUS_DATAPROPERTIESOWLWRITER_H

#include <string>

#include "ontologenius/core/ontologyIO/Owl/writers/PropertiesOwlWriter.h"

namespace ontologenius {

  class DataPropertyGraph;
  class DataPropertyBranch;

  class DataPropertiesOwlWriter : public PropertiesOwlWriter<DataPropertyBranch>
  {
  public:
    DataPropertiesOwlWriter(DataPropertyGraph* property_graph, const std::string& ns);
    ~DataPropertiesOwlWriter() = default;

    void write(FILE* file);

  private:
    DataPropertyGraph* property_graph_;

    void writeProperty(DataPropertyBranch* branch);
    void writeSubPropertyOf(DataPropertyBranch* branch);
    void writeRange(DataPropertyBranch* branch);
    void writeDomain(DataPropertyBranch* branch);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTIESOWLWRITER_H
