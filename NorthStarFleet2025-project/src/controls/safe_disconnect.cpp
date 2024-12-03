#include "control\safe_disconnect.hpp"

#include "tap/control/command_scheduler.hpp"
#include "tap/drivers.hpp"

namespace src::control
{
RemoteSafeDisconnectFunction::RemoteSafeDisconnectFunction(tap::Drivers *drivers)
{
    this->drivers = drivers;
}
bool RemoteSafeDisconnectFunction::operator()() { return !drivers->remote.isConnected(); }
}  // namespace src::control