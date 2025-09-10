#ifndef TEST_BED_DRIVERS_HPP_
#define TEST_BED_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"

// #include "src/mock/turret_mcb_can_comm_mock.hpp"

#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"
#include "tap/motor/sparkmax/rev_motor_tx_handler.hpp"

#include "communication/can/turret/turret_mcb_can_comm.hpp"
#include "communication/serial/vision_comms.hpp"
#include "robot/control_operator_interface.hpp"

#endif

namespace src::testbed
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
          controlOperatorInterface(this),
          visionComms(this),
          turretMCBCanCommBus2(this, tap::can::CanBus::CAN_BUS2)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus2;
#else
public:
    control::ControlOperatorInterface controlOperatorInterface;
    serial::VisionComms visionComms;
    can::TurretMCBCanComm turretMCBCanCommBus2;
#endif
};  // class src::testbedDrivers
}  // namespace src::testbed

#endif  // testbed_DRIVERS_HPP_
