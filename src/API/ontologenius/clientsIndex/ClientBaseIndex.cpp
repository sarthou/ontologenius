#include "ontologenius/API/ontologenius/clientsIndex/ClientBaseIndex.h"

#include <cstddef>
#include <cstdint>

namespace onto {

  size_t ClientBaseIndex::cpt = 0;
  int16_t ClientBaseIndex::ignore;
  bool ClientBaseIndex::client_verbose = false;

} // namespace onto
