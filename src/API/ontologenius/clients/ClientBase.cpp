#include "ontologenius/API/ontologenius/clients/ClientBase.h"

#include <cstddef>
#include <cstdint>

namespace onto {

  size_t ClientBase::cpt = 0;
  int16_t ClientBase::ignore;
  bool ClientBase::client_verbose = false;

} // namespace onto