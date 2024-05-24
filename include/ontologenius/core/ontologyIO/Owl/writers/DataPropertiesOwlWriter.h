#ifndef ONTOLOGENIUS_DATAPROPERTIESOWLWRITER_H
#define ONTOLOGENIUS_DATAPROPERTIESOWLWRITER_H

#include <string>

#include "ontologenius/core/ontologyIO/Owl/writers/PropertiesOwlWriter.h"

namespace ontologenius {

  class DataPropertyGraph;
  class DataPropertyBranch_t;

  class DataPropertiesOwlWriter : public PropertiesOwlWriter<DataPropertyBranch_t>
  {
  public:
    DataPropertiesOwlWriter(DataPropertyGraph* property_graph, const std::string& ns);
    ~DataPropertiesOwlWriter(){};

    void write(FILE* file);

  private:
    DataPropertyGraph* property_graph_;

    void writeProperty(DataPropertyBranch_t* branch);
    void writeSubPropertyOf(DataPropertyBranch_t* branch);
    void writeRange(DataPropertyBranch_t* branch);
    void writeDomain(DataPropertyBranch_t* branch);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_DATAPROPERTIESOWLWRITER_H
