#include "ontologenius/API/ontologenius/clients/ClientBase.h"

namespace onto {

size_t ClientBase::cpt = 0;
int16_t ClientBase::ignore_;
bool ClientBase::verbose_ = false;

} // namespace onto