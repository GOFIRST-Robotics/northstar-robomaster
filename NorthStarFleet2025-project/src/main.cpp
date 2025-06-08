/*
 * Copyright (c) 2020-2021 NorthStart
 *
 * This file is part of NorthStarFleet2025.
 *
 * NorthStarFleet2025 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NorthStarFleet2025 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NorthStarFleet2025.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "communication/serial/fly_sky.hpp"

#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "robot/robot_control.hpp"

/* robot includes ---------------------------------------------------------*/

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::arch::PeriodicMilliTimer revTxPublisherTimeout(20);

#ifdef TARGET_STANDARD
using namespace src::standard;
#elif TARGET_SENTRY
using namespace src::sentry;
#elif TARGET_HERO
using namespace src::hero;
#elif TURRET
#include "communication/can/chassis/chassis_mcb_can_comm.hpp"
using namespace src::gyro;
ChassisMcbCanComm chassisMcbCanComm(DoNotUse_getDrivers());
#elif TARGET_TEST_BED
using namespace src::testbed;
#endif

// using namespace std::chrono_literals;

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(Drivers *drivers);
int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    Drivers *drivers = DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    initSubsystemCommands(drivers);
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->bmi088.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
#ifdef TURRET
            PROFILE(drivers->profiler, chassisMcbCanComm.sendIMUData, ());
            PROFILE(drivers->profiler, chassisMcbCanComm.sendSynchronizationRequest, ());
#else
            // PROFILE(drivers->profiler, drivers->turretMCBCanCommBus2.sendData, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
#endif
        }
#if defined(TARGET_STANDARD) || defined(TARGET_SENTRY) || defined(TARGET_TEST_BED)
        if (revTxPublisherTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->revMotorTxHandler.encodeAndSendCanData, ());
        }
#endif

        // if(!drivers->turretMCBCanCommBus2.isConnected()){
        //     std::cout<<"poop";
        // }
        modm::delay_us(10);
    }
    return 0;
}
static void initializeIo(Drivers *drivers)
{
    drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>();
    drivers->can.initialize();
    drivers->leds.init();
    drivers->digital.init();
    drivers->pwm.init();
    drivers->errorController.init();
    drivers->terminalSerial.initialize();
    drivers->bmi088.initialize(500, .001, 0);

#if defined(TARGET_STANDARD) || defined(TARGET_HERO)
    drivers->turretMCBCanCommBus2.init();
#endif
#ifdef TURRET
    chassisMcbCanComm.init();
#else
    drivers->analog.init();
    drivers->remote.initialize();
    drivers->refSerial.initialize();
    // drivers->vissionComs.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
#endif
}
float debugYaw = 0.0f;
float debugPitch = 0.0f;
float debugRoll = 0.0f;
float debugYawV = 0.0f;
float debugPitchV = 0.0f;
float debugRollV = 0.0f;
bool conneccc = false;

bool cal = false;
static void updateIo(Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->bmi088.read();

#ifndef TURRET
    drivers->refSerial.updateSerial();
    // drivers->vissionComs.updateSerial();
    drivers->remote.read();
    if (cal)
    {
        cal = false;
        drivers->bmi088.requestCalibration();
    }
    debugYawV = drivers->bmi088.getGz();
    debugYaw = modm::toDegree(drivers->bmi088.getYaw());
    debugPitchV = drivers->bmi088.getGy();
    debugPitch = modm::toDegree(drivers->bmi088.getPitch());
    debugRollV = drivers->bmi088.getGx();
    debugRoll = modm::toDegree(drivers->bmi088.getRoll());
    conneccc = drivers->remote.isConnected();
#endif
}