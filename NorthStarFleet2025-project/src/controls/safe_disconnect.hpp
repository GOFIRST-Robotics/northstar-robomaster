#ifndef SAFE_DISCONNECT_HPP_
#define SAFE_DISCONNECT_HPP_

#include "tap/control/command_scheduler.hpp"
#include "tap/drivers.hpp"

/**
 * Defines the condition for a robot to be "safely disconnected" to be
 * when the remote is disconnected. Ends running of all current Commands and
 * disallows new Commands from being added.
 */
namespace src::control
{
class RemoteSafeDisconnectFunction : public tap::control::SafeDisconnectFunction
{
public:
    RemoteSafeDisconnectFunction(tap::Drivers *drivers);
    virtual bool operator()();

private:
    tap::Drivers *drivers;
};
}  // namespace src::control

#endif  // SAFE_DISCONNECT_HPP_
