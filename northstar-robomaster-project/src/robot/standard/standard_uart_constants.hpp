
#ifndef UART_CONSTANTS_HPP_
#error "Do not include this file directly! Use uart_constants.hpp instead."
#endif

#ifndef STANDARD_UART_CONSTANTS_HPP
#define STANDARD_UART_CONSTANTS_HPP

#include <cstdint>

#include "tap/architecture/periodic_timer.hpp"

namespace src::serial
{
static constexpr uint32_t TIME_BEFORE_UART_START =
    1000;  // initial delay of 1 second to allow time for things to start up
tap::arch::MilliTimeout messageOffsetInitializationTimeout{TIME_BEFORE_UART_START};

/** Time in ms between sending the odometry message. */
static constexpr uint32_t TIME_BTWN_SENDING_ODOMETRY_MSG = 0.025E3;
static constexpr uint32_t TIME_BEFORE_SENDING_ODOMETRY_MSG = 0.5E3 + TIME_BEFORE_UART_START;
tap::arch::PeriodicMilliTimer sendOdometryMsgTimeout{TIME_BTWN_SENDING_ODOMETRY_MSG};

/** Time in ms between sending the ref system data message. */
static constexpr uint32_t TIME_BTWN_SENDING_REF_MSG = 0.5E3;
static constexpr uint32_t TIME_BEFORE_SENDING_REF_MSG = 0.125E3 + TIME_BEFORE_UART_START;
tap::arch::PeriodicMilliTimer sendRefMsgTimeout{TIME_BTWN_SENDING_REF_MSG};

/** Time in ms between sending the Robot ID message. */
static constexpr uint32_t TIME_BTWN_SENDING_ROBOT_ID_MSG = 10.25E3;
static constexpr uint32_t TIME_BEFORE_SENDING_ROBOT_ID_MSG = 0.125E3 + TIME_BEFORE_UART_START;
tap::arch::PeriodicMilliTimer sendRobotIDMsgTimeout{TIME_BTWN_SENDING_ROBOT_ID_MSG};

}  // namespace src::serial
#endif