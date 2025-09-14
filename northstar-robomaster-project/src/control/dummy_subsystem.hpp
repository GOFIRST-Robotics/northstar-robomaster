#ifndef DUMMY_SUBSYSTEM_HPP_
#define DUMMY_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

class DummySubsystem : public tap::control::Subsystem
{
public:
    DummySubsystem(tap::Drivers *drivers) : tap::control::Subsystem(drivers) {}
};

#endif  // DUMMY_SUBSYSTEM_HPP_
